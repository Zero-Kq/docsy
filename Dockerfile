FROM floryn90/hugo:ext-alpine

# 切换到root用户执行权限操作（安装依赖）
USER root

# 1. 安装git（Docsy模块依赖）、nodejs+npm（前端依赖rtlcss/postcss）
# 2. 安装全局的postcss-cli和rtlcss（解决样式转换报错）
# 3. 配置git安全目录（避免权限警告）
RUN apk add --no-cache git nodejs npm \
  && npm install -g postcss-cli rtlcss \
  && git config --global --add safe.directory /src

# 可选：切回原镜像的非root用户（提升容器安全性）
USER hugo

# 暴露Hugo默认端口（可选，增强可读性）
EXPOSE 1313

# 设置工作目录（与docker-compose的volume挂载路径对齐）
WORKDIR /src

COPY . /src
# 补充：执行 Hugo 构建（生成 public 目录）
RUN hugo --destination public