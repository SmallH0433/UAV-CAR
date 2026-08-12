import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "ASTRA 空地联合任务台",
  description: "无人机、Hunter 无人车、Gazebo/SITL 与 ROS 2 联合任务控制和状态可视化。",
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
