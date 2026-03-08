# 博客完善计划

## 需求分析

根据用户需求，需要完成以下任务：

1. **创建备份分支** - 将当前内容创建为 git 仓库新分支作为备份
2. **删除 aaa 文件夹** - 删除 `content/cn/aaa` 目录
3. **统一背景图片** - 博客列表页背景图片与主页保持一致
4. **统一内容页风格** - 博客文章页、说明页、关于页等与博客列表页保持风格一致

## 当前状态分析

### 背景图片配置

| 页面 | 当前背景 | 配置位置 |
|------|---------|---------|
| 主页 | `/docsy/images/backgrounds/bg11.webp` | `_variables_project.scss` |
| 博客列表页 | `/docsy/images/backgrounds/wallhaven-odr71m.jpg` | `hugo.yaml` → `blogBackground` |
| 博客文章页 | `/docsy/images/backgrounds/wallhaven-odr71m.jpg` | `layouts/blog/single.html` |
| 标签/分类页 | `/docsy/images/blog/blog_background.jpg` | `layouts/taxonomy/*.html` |

### 风格差异

- **主页/关于页**: 使用 `blocks/cover` 组件，全屏背景 + 毛玻璃效果
- **博客列表页**: 自定义背景 + 白色半透明遮罩 + 毛玻璃卡片
- **博客文章页**: 与列表页类似，但侧边栏文字颜色不同
- **说明文档页**: 无特殊背景，默认样式

## 实施计划

### 步骤 1: 创建备份分支

```bash
cd d:\WORK\Docsy\docsy
git checkout -b backup/original-content
git push origin backup/original-content
git checkout main
```

### 步骤 2: 删除 aaa 文件夹

删除以下文件：
- `content/cn/aaa`

### 步骤 3: 统一背景图片

修改 `hugo.yaml` 中的 `blogBackground` 参数：
```yaml
blogBackground: '/docsy/images/backgrounds/bg11.webp'
```

修改 `layouts/taxonomy/tag.html` 和 `layouts/taxonomy/category.html` 中的背景图片路径。

### 步骤 4: 统一内容页风格

确保以下页面使用一致的背景和样式：
- 博客文章页 (`layouts/blog/single.html`) - 已使用 `blogBackground`
- 说明文档页 - 需要添加背景样式
- 关于页 - 已使用 `blocks/cover`，风格一致

## 文件修改清单

| 文件 | 操作 |
|------|------|
| `hugo.yaml` | 修改 `blogBackground` 参数 |
| `content/cn/aaa/_index.md` | 删除 |
| `layouts/taxonomy/tag.html` | 修改背景图片路径 |
| `layouts/taxonomy/category.html` | 修改背景图片路径 |

## 预期效果

1. 所有内容页面使用统一的背景图片 (`bg11.webp`)
2. 风格保持一致：毛玻璃效果 + 半透明遮罩
3. aaa 测试文件夹已删除
4. 原始内容已备份到 `backup/original-content` 分支
