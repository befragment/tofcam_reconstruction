import numpy as np
import cv2
import open3d as o3d
from scipy.ndimage import median_filter
import tempfile

min_range_mm, max_range_mm = 300, 4500
mm2m = 1.0 / 1000.0


class CameraConfig:
    width, height = 640, 480
    fov_h = np.deg2rad(70.0)
    fov_v = np.deg2rad(50.0)
    fx = (width / 2) / np.tan(fov_h / 2)
    fy = (height / 2) / np.tan(fov_v / 2)
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0

class FiltersConfig:
    median_ksize = 3
    bilateral_d = 5
    bilateral_sigma_color = 60
    bilateral_sigma_space = 60
    flying_kernel = 3
    flying_mad_scale = 3.5
    hole_close_kernel = 3

class CloudMeshConfig:
    voxel_size = 0.0075
    nb_neighbors = 30
    std_ratio = 2.0
    rad_outlier_radius = 0.04
    rad_outlier_min_pts = 16

    nn_radius = 0.015
    nn_max_nn = 50

    poisson_depth = 10
    trim_low_quantile = 0.05
    alpha_multiplier = 1.5


def _auto_alpha_from_cloud(cloud: o3d.geometry.PointCloud, 
                           k: int = 12, 
                           mult: float = CloudMeshConfig.alpha_multiplier) -> float:
    pcd_kdtree = o3d.geometry.KDTreeFlann(cloud)
    dists = []
    pts = np.asarray(cloud.points)
    if len(pts) == 0:
        return 0.02
    step = max(1, len(pts) // 8000)
    for i in range(0, len(pts), step):
        _, idx, dist2 = pcd_kdtree.search_knn_vector_3d(cloud.points[i], k)
        if len(dist2) > 1:
            d = float(np.sqrt(dist2[-1]))
            dists.append(d)
    return float(np.mean(dists) * mult) if dists else 0.02


def process_3d_image(depth_bytes: bytes) -> o3d.geometry.TriangleMesh:
    """Принимает бинарное содержимое .depth (uint16, мм, размер CameraConfig.width x CameraConfig.height)
    и возвращает TriangleMesh, построенный методом Alpha Shape **с vertex_colors**, чтобы .ply был цветным в Open3D.
    """
    W, H = CameraConfig.width, CameraConfig.height
    fx, fy, cx, cy = CameraConfig.fx, CameraConfig.fy, CameraConfig.cx, CameraConfig.cy

    depth = np.frombuffer(depth_bytes, dtype=np.uint16)
    expected = W * H
    if depth.size != expected:
        raise ValueError(f"Неверный размер .depth: {depth.size}, ожидалось {expected}")
    depth = depth.reshape((H, W))

    # Валидные расстояния
    valid = (depth >= min_range_mm) & (depth <= max_range_mm)

    # Фильтрация
    depth_f = median_filter(depth, size=FiltersConfig.median_ksize)
    depth32 = depth_f.astype(np.float32)
    depth32 = cv2.bilateralFilter(
        depth32,
        d=FiltersConfig.bilateral_d,
        sigmaColor=FiltersConfig.bilateral_sigma_color,
        sigmaSpace=FiltersConfig.bilateral_sigma_space,
    )

    # подавление «летающих пикселей» по локальной MAD
    k = FiltersConfig.flying_kernel
    pad = k // 2
    depth_pad = np.pad(depth32, pad, mode='edge')
    clean = depth32.copy()
    for y in range(H):
        for x in range(W):
            if not valid[y, x]:
                continue
            roi = depth_pad[y:y+k, x:x+k]
            med = np.median(roi)
            mad = np.median(np.abs(roi - med)) + 1e-6
            if np.abs(depth32[y, x] - med) > FiltersConfig.flying_mad_scale * mad:
                clean[y, x] = med
    depth32 = clean

    # Морфологическое закрытие дыр
    valid = (depth32 >= min_range_mm) & (depth32 <= max_range_mm)
    mask_u8 = (valid.astype(np.uint8) * 255)
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (FiltersConfig.hole_close_kernel, FiltersConfig.hole_close_kernel))
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, se)
    valid = mask_u8 > 0

    # В метры
    depth_m = depth32 * mm2m
    depth_m[~valid] = 0.0

    # Проекция в XYZ
    u_grid, v_grid = np.meshgrid(np.arange(W), np.arange(H))
    Z = depth_m[valid]
    if Z.size == 0:
        raise ValueError("Нет валидных глубин после фильтрации")
    X = (u_grid[valid] - cx) * Z / fx
    Y = (v_grid[valid] - cy) * Z / fy

    points = np.column_stack((X, Y, Z))
    pcd_raw = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    # Downsample
    pcd = pcd_raw.voxel_down_sample(voxel_size=CloudMeshConfig.voxel_size)

    # Очистка облака
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=CloudMeshConfig.nb_neighbors, std_ratio=CloudMeshConfig.std_ratio)
    pcd, _ = pcd.remove_radius_outlier(nb_points=CloudMeshConfig.rad_outlier_min_pts, radius=CloudMeshConfig.rad_outlier_radius)

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=CloudMeshConfig.nn_radius, max_nn=CloudMeshConfig.nn_max_nn))
    pcd.orient_normals_consistent_tangent_plane(k=10)

    # Alpha Shape (с попыткой авто-подбора и fallback)
    alpha = _auto_alpha_from_cloud(pcd)
    try:
        mesh_alpha = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    except Exception:
        mesh_alpha = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha * 1.5)
    mesh_alpha.compute_vertex_normals()

    return mesh_alpha

def convert_depth_to_ply(depth_bytes: bytes) -> bytes:
    """Конвертирует .depth → .ply (Alpha-shape mesh, с цветом) и возвращает байты файла .ply."""
    mesh = process_3d_image(depth_bytes)

    with tempfile.NamedTemporaryFile(suffix=".ply") as tmp:
        ok = o3d.io.write_triangle_mesh(tmp.name, mesh, write_ascii=False)
        if not ok:
            raise RuntimeError("Не удалось записать PLY")
        tmp.seek(0)
        return tmp.read()