<!--
Language selector: default is English. To view Chinese, click the 中文 link.
-->

<p align="right">Other Language: <a href="README.zh-CN.md">简体中文</a></p>

This is a Jacobian-based IK Solver running in Unity, derived from the [AR2-D2 project](https://github.com/jiafei1224/AR2-D2) by [jiafei1224](https://github.com/jiafei1224). A huge thank you to their team for their incredible work. Originally designed for the Franka Emika Panda arm (files in `Panda/`), I have successfully adapted it for the UR3e robotic arm, which is now performing well (files in `UR/`). I’m sharing my modifications here in hopes that this might help if you're looking for a Jacobian-based IK solver. I have also prepared an [AI adaptation prompt](./ADAPTATION_GUIDE.md) that you can directly provide to an AI assistant to help you make customized adjustments for your own setup.

<p align="center">
  <img src="https://img.shields.io/badge/Unity-100000?style=for-the-badge&logo=unity&logoColor=white" alt="Unity">
  <img src="https://img.shields.io/badge/C%23-239120?style=for-the-badge&logo=c-sharp&logoColor=white" alt="C#">
</p>

<p align="center">
  <img src="https://img.shields.io/github/languages/top/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity?style=flat-square" alt="Language Stats">
  <img src="https://img.shields.io/github/languages/code-size/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity?style=flat-square" alt="Code Size">
</p>

<p align="center">
  <img src="https://img.shields.io/github/license/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity?style=flat-square" alt="License">
  <img src="https://img.shields.io/github/last-commit/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity?style=flat-square" alt="Last Commit">
</p>
 
## Demo

The `demo/` folder contains an AR pick-and-place demo (see `demo/ar_pick_place.mp4`) that uses the IK Solver implemented in this repository. In other words, the IK Solver used by the demo is this repository's IK Solver.

<!-- Embedded demo video: if playback fails, the links below provide direct download/view -->
<video controls width="640">
  <source src="https://raw.githubusercontent.com/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity/main/demo/ar_pick_place.mp4" type="video/mp4">
  Your browser does not support the video tag. You can download it from
  <a href="https://raw.githubusercontent.com/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity/main/demo/ar_pick_place.mp4">raw file</a> or view the `demo/` folder.
</video>
