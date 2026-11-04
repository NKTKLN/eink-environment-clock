# Third-party skills

Copied unchanged from upstream. Do not edit these files: an update overwrites them.

| Skill | Purpose | Source | License |
|---|---|---|---|
| `frontend-design` | Visual direction, typography, layout for new or reworked UI | [anthropics/skills](https://github.com/anthropics/skills/tree/main/skills/frontend-design) @ `ca45fc4` | Apache 2.0, `frontend-design/LICENSE.txt` |
| `webapp-testing` | Playwright scripts for testing a local web app, screenshots, console logs | [anthropics/skills](https://github.com/anthropics/skills/tree/main/skills/webapp-testing) @ `ca45fc4` | Apache 2.0, `webapp-testing/LICENSE.txt` |
| `web-design-guidelines` | Review UI code against Vercel's Web Interface Guidelines (fetched at run time) | [vercel-labs/agent-skills](https://github.com/vercel-labs/agent-skills/tree/main/skills/web-design-guidelines) @ `1a2c79d` | MIT |

Project-specific design rules (palette, fonts, tone) live in the coursework brand book
(`01-brand/brand.md` next to this repository) and take precedence over the generic
advice in these skills.

## Updating

```sh
git clone --depth 1 https://github.com/anthropics/skills.git /tmp/anthropic-skills
cp -r /tmp/anthropic-skills/skills/{frontend-design,webapp-testing} .claude/skills/
```

```sh
git clone --depth 1 https://github.com/vercel-labs/agent-skills.git /tmp/vercel-skills
cp -r /tmp/vercel-skills/skills/web-design-guidelines .claude/skills/
```
