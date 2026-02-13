这是一个运行在Unity上的Jacobian based IK Solver, 来自[jiafei1224](https://github.com/jiafei1224)的[AR2-D2 project](https://github.com/jiafei1224/AR2-D2)，非常感谢他们团队的工作，他们真的做的非常棒。其代码原本是一个给Franka Emika Panda机械臂设计的IK Solver（代码在`Panda/`），我成功的将其适配到了UR3e机械臂上，目前运行的不错（代码在`UR/`）。我在此分享我做出的改动。若是您需要一个Jacobian based IK Solver, 我的经历或许可以帮到你。我制作了一个[prompt](./ADAPTATION_GUIDE.md)，可以直接扔给AI, 让AI帮助您做出针对性改动。 

<!--
Language selector: default is English. Click the English link to view the English version.
-->

<p align="right">其他语言: <a href="README.md">English</a></p>

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

## 演示 (Demo)

`demo/` 文件夹包含了一个 AR 拾放（pick-and-place）演示（参见 `demo/ar_pick_place.mp4`），该演示中使用的 IK Solver 就是本仓库中的 IK Solver。

<!-- 内嵌演示视频：若无法播放，请使用下方链接下载或前往 demo/ 查看 -->
<!-- 预览 GIF，点击将打开 raw MP4（请在本地生成 demo/preview.gif 并推送） -->
[![演示预览](https://raw.githubusercontent.com/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity/main/demo/preview.gif)](https://raw.githubusercontent.com/wjswjsss/Adaptable-Robot-Jacobian-IK-Unity/main/demo/ar_pick_place.mp4)

若预览 GIF 尚未存在，上方链接将打开 raw MP4；你也可以直接前往 `demo/` 文件夹查看。
