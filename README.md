---
title: "Robotics - Collaboration Guide"
description: "Contributing guide for Robotics course content"
tableOfContents: true
sidebar:
  order: 999
---

# Robotics

![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-blue)
![Contributors Welcome](https://img.shields.io/badge/contributors-welcome-orange)

**Read this course at:** [https://siliconwit.com/education/robotics/](https://siliconwit.com/education/robotics/)

A course on robot kinematics and manipulation, covering robot arm geometry, forward and inverse kinematics, quaternion orientation, Jacobian velocity analysis, trajectory planning, and Python simulation. Each lesson pairs rigorous theory with working code so you can visualize and verify every concept.

## Lessons

| # | Title |
|---|-------|
| 1 | Robot Arm Geometry and Configuration |
| 2 | Forward and Inverse Kinematics |
| 3 | Orientation and Quaternions |
| 4 | Velocity Kinematics and the Jacobian |
| 5 | Trajectory Planning and Motion Control |
| 6 | Robot Simulation and Practical Applications |

## File Structure

```
robotics/
├── index.mdx
├── robot-arm-geometry-configuration.mdx
├── forward-inverse-kinematics.mdx
├── orientation-quaternions.mdx
├── velocity-kinematics-jacobian.mdx
├── trajectory-planning-motion-control.mdx
├── robot-simulation-applications.mdx
└── README.md
```

## How to Contribute

All commands below work on Linux, macOS, and Windows (using Git Bash, PowerShell, or Command Prompt with Git installed).

### For Team Members (with push access)

**First time setup (clone the repo once):**

```bash
git clone https://github.com/SiliconWit/robotics.git
cd robotics
```

**Every time you start working:**

```bash
git pull origin main
```

Always pull before making changes. This avoids conflicts with other contributors.

**After making your changes:**

```bash
git add .
git commit -m "Brief description of what you changed"
git push origin main
```

**If you get a push error** (someone pushed before you):

```bash
git pull origin main
```

Git will merge the changes automatically in most cases. If there is a conflict, Git will mark the conflicting lines in the file. Open the file, choose which version to keep, then:

```bash
git add .
git commit -m "Resolve merge conflict"
git push origin main
```

**Tips to avoid conflicts:**

- Always `git pull origin main` before you start working
- Push your changes as soon as you are done, do not hold onto uncommitted work for long
- Coordinate with other contributors so two people are not editing the same file at the same time

### For External Contributors (without push access)

1. Fork the repository: [SiliconWit/robotics](https://github.com/SiliconWit/robotics)
2. Clone your fork:
   ```bash
   git clone https://github.com/YOUR-USERNAME/robotics.git
   cd robotics
   ```
3. Make your changes and commit:
   ```bash
   git add .
   git commit -m "Brief description of what you changed"
   git push origin main
   ```
4. Open a Pull Request against `main` on the original repository
5. Describe what you changed and why in the PR description

## Content Standards

- All lesson files use `.mdx` format
- `<BionicText>` may be used in later content sections but not in lesson intro paragraphs
- Code blocks should include a title attribute:
  ````mdx
  ```python title="forward_kinematics.py"
  import numpy as np
  T = dh_matrix(theta, d, a, alpha)
  ```
  ````
- Use Starlight components (`<Tabs>`, `<TabItem>`, `<Steps>`, `<Card>`) where appropriate
- Keep paragraphs concise and focused on practical application
- Include working Python examples that readers can run directly
- Mathematical notation uses LaTeX in MDX

## Local Development

Clone the main site repository and initialize submodules:

```bash
git clone --recurse-submodules <main-repo-url>
cd siliconwit-com
npm install
npm run dev
```

To test a production build:

```bash
npm run build
```

## License

This course content is released under the [MIT License](LICENSE).
