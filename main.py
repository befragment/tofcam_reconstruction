from io import BytesIO

from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware

from threedprocessing.processing import convert_depth_to_ply

app = FastAPI(title="Depth→PLY API", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/convert")
async def convert_endpoint(
    file: UploadFile = File(..., description="Binary .depth file to convert")
):
    """Accept a .depth file and return a .ply file as an attachment."""
    filename = file.filename or "uploaded.depth"
    if not filename.lower().endswith(".depth"):
        raise HTTPException(status_code=400, detail="Expected a .depth file upload")

    try:
        depth_bytes = await file.read()
        if not depth_bytes:
            raise HTTPException(status_code=400, detail="Uploaded file is empty")

        ply_bytes = convert_depth_to_ply(depth_bytes)

    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as exc:
        # Hide internals, but log if you have logging configured
        raise HTTPException(status_code=500, detail="Failed to convert process .depth file") from exc

    # Build a safe output filename
    out_name = filename.rsplit("/", 1)[-1].rsplit("\\", 1)[-1]
    out_name = out_name.rsplit(".", 1)[0] + ".ply"

    return StreamingResponse(
        BytesIO(ply_bytes),
        media_type="application/octet-stream",
        headers={
            "Content-Disposition": f"attachment; filename=\"{out_name}\"",
            "X-File-Source": "depth-to-ply",
        },
    )


# Run with: uvicorn main:app --reload
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)

