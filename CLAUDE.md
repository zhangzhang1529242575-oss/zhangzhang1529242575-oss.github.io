# myblog — 个人网站仓库

## 发布规则（重要）

- **任何 `git push` 之前，必须先向用户展示本次改动并取得明确确认**：
  - `git status --porcelain`
  - `git diff --stat`（未提交时）或 `git show --stat HEAD`（已提交未推送时）
  - 一段简短说明：改了哪些文件、影响哪个页面
- 未得到确认前，只允许本地 `git add` / `git commit`，不允许 `git push`。
- `../motor-research-network/scripts/sync.js` 默认**只提交、不推送**；只有确认后才加 `--push` 运行。

## 站点结构

- Hugo + PaperMod，`main` 分支 push 后由 `.github/workflows/deploy.yml` 自动构建发布到 GitHub Pages。
- 线上地址：<https://zhangzhang1529242575-oss.github.io/>
- 研究网络页面：`static/motor-network/index.html`（由 `../motor-research-network` 生成，不要直接改）。

## 研究网络维护

源数据在 `../motor-research-network/assets/data.js`，流程见该目录 `README.md` 第 9 节。
