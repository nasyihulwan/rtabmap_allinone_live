// Konversi ROS (x,y,z, z=atas) -> Three.js (x,y,z, y=atas): three(x,z,-y) = ros(x,y,z).
// Dipakai di semua viewer 3D (RobotViewer, PointCloudViewer) supaya konsisten —
// satu bug di sini akan salah-arah di SEMUA viewer sekaligus kalau diketik ulang beda-beda.
export function rosToThree([x, y, z]: [number, number, number]): [number, number, number] {
  return [x, z, -y]
}
