FROM python:3.12-slim-bookworm
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
    NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    ROBOT_DATA_DIR=/data ROBOT_VIEW_FPS=5 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 libgl1 libglvnd0 libgles2 libglib2.0-0 libx11-6 libxext6 stockfish gosu \
    && rm -rf /var/lib/apt/lists/*
RUN printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
    > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
WORKDIR /app
COPY requirements.lock /app/requirements.lock
RUN pip install --no-cache-dir -r requirements.lock
COPY chess_sim /app/chess_sim
COPY robot_web /app/robot_web
COPY deploy/entrypoint.sh /app/entrypoint.sh
RUN useradd --uid 1000 --create-home robot && mkdir -p /data \
    && chown -R robot:robot /app /data && chmod +x /app/entrypoint.sh
EXPOSE 8098
LABEL org.opencontainers.image.source="https://github.com/ForceConstant/robot-arm-sim" \
      org.opencontainers.image.version="1.4.1"
HEALTHCHECK --start-period=60s --interval=15s --timeout=5s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8098/health',timeout=3)"
ENTRYPOINT ["/app/entrypoint.sh"]
