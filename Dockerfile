FROM floryn90/hugo:ext-alpine

# 切换到root用户执行需要权限的操作
USER root

RUN apk add git && \
  git config --global --add safe.directory /src
  
# 可选：切回原镜像的非root用户（提升安全性）
USER hugo
