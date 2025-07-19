class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  clone() { return new Vector3(this.x, this.y, this.z); }
  length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }
  normalize() {
    const len = this.length();
    return len === 0 ? this.clone() : this.multiplyScalar(1 / len);
  }
}

function applyFullTransform(bone, bindpose) {
  const { position, rotation, scale } = bone;
  let x = position.x * scale.x;
  let y = position.y * scale.y;
  let z = position.z * scale.z;
  const qx = rotation.x, qy = rotation.y, qz = rotation.z, qw = rotation.w;
  const ix =  qw * x + qy * z - qz * y;
  const iy =  qw * y + qz * x - qx * z;
  const iz =  qw * z + qx * y - qy * x;
  const iw = -qx * x - qy * y - qz * z;
  const rx = ix * qw + iw * -qx + iy * -qz - iz * -qy;
  const ry = iy * qw + iw * -qy + iz * -qx - ix * -qz;
  const rz = iz * qw + iw * -qz + ix * -qy - iy * -qx;
  return {
    x: bindpose.e00 * rx + bindpose.e01 * ry + bindpose.e02 * rz + bindpose.e03,
    y: bindpose.e10 * rx + bindpose.e11 * ry + bindpose.e12 * rz + bindpose.e13,
    z: bindpose.e20 * rx + bindpose.e21 * ry + bindpose.e22 * rz + bindpose.e23
  };
}

class Kalman1D {
  constructor(r = 0.01, q = 0.1) {
    this.r = r; this.q = q;
    this.p = 1; this.x = 0;
  }
  update(measurement) {
    this.p += this.q;
    const k = this.p / (this.p + this.r);
    this.x += k * (measurement - this.x);
    this.p *= (1 - k);
    return this.x;
  }
}

function sendInputToMouse({ deltaX, deltaY }) {
  console.log(`🎯 Adjust Mouse → ΔX=${deltaX.toFixed(4)} | ΔY=${deltaY.toFixed(4)}`);
}

// Tham số weaponDragConfig cho từng loại súng
const weaponDragConfig = {
  default: { smooth: 0.7, snapRadius: 360, strongPullRadius: 400 },
  mp40: { smooth: 1.0, snapRadius: 360, strongPullRadius: 420 },
  m1887: { smooth: 1.2, snapRadius: 360, strongPullRadius: 450 },
  ak: { smooth: 0.8, snapRadius: 360, strongPullRadius: 400 }
};
const GamePackages = {
  GamePackage1: "com.dts.freefireth",
  GamePackage2: "com.dts.freefiremax"
};
// Hàm chính drag về phía head không vượt quá target, tăng lực hút theo weaponType

function dragToBoneHead(
  cameraPos, boneHead, bindpose,
  kalmanYaw, kalmanPitch,
  crosshair, weaponType = "default"
) {
  const headWorld = applyFullTransform(boneHead, bindpose);
  const dir = new Vector3(
    headWorld.x - cameraPos.x,
    headWorld.y - cameraPos.y,
    headWorld.z - cameraPos.z
  ).normalize();

  // Gốc ngắm
  const pitch = -Math.asin(dir.y);
  const yaw = Math.atan2(dir.x, dir.z);

  // Kalman lọc mượt
  const smoothYaw = kalmanYaw.update(yaw);
  const smoothPitch = kalmanPitch.update(pitch);

  // Cấu hình theo vũ khí
  const config = weaponDragConfig[weaponType] || weaponDragConfig["default"];
  const sensitivity = config.sensitivity || { x: 0.52, y: 0.49 };
  const maxPullSpeed = config.maxSpeed || 1.0;
  const dragSmooth = config.smooth || 0.6;
  const snapRadius = config.snapRadius || 0.07;
  const strongPullRadius = config.strongPullRadius || 0.2;
  const maxRadius = config.maxRadius || 0.03;

  // Tính offset hiện tại từ crosshair → head
  const dx = headWorld.x - crosshair.x;
  const dy = headWorld.y - crosshair.y;
  const dist = Math.hypot(dx, dy);

  // Nếu nằm trong snap zone
  if (dist < snapRadius) {
    crosshair.x = headWorld.x;
    crosshair.y = headWorld.y;
    return sendInputToMouse({ deltaX: dx, deltaY: dy });
  }

  // Tính easing theo vùng hút mạnh
  let ease;
  if (dist < strongPullRadius) {
    const factor = 1 + (strongPullRadius - dist) / strongPullRadius;
    ease = Math.min(1, dragSmooth * (dist / maxPullSpeed) * factor);
  } else {
    ease = Math.min(1, dragSmooth * (dist / maxPullSpeed));
  }

  // Kéo mới tới gần head
  let newX = crosshair.x + dx * ease * sensitivity.x;
  let newY = crosshair.y + dy * ease * sensitivity.y;

  // Clamp không cho vượt quá vùng quanh head
  const offsetX = newX - headWorld.x;
  const offsetY = newY - headWorld.y;
  const offsetDist = Math.hypot(offsetX, offsetY);

  if (offsetDist > maxRadius) {
    newX = headWorld.x + (offsetX / offsetDist) * maxRadius;
    newY = headWorld.y + (offsetY / offsetDist) * maxRadius;
  }

  const deltaX = newX - crosshair.x;
  const deltaY = newY - crosshair.y;

  crosshair.x = newX;
  crosshair.y = newY;

  sendInputToMouse({ deltaX, deltaY });
}

const kalmanYaw = new Kalman1D(0.005, 0.01);
const kalmanPitch = new Kalman1D(0.005, 0.01);

const camera = { position: { x: 0, y: 1.7, z: 0 } };
const crosshair = { x: 0, y: 0 };

const enemy = {
  bones: {
    bone_Head: {
      position: { x: -0.0457, y: -0.0044, z: -0.0200 },
      rotation: { x: 0.0258174837, y: -0.08611039, z: -0.1402113, w: 0.9860321 },
      scale: { x: 0.99999994, y: 1.00000012, z: 1.0 }
    }
  },
  bindpose: {
    e00: -1.3456e-13, e01: 8.88e-14, e02: -1.0, e03: 0.487912,
    e10: -2.84e-6, e11: -1.0, e12: 8.88e-14, e13: -2.84e-14,
    e20: -1.0, e21: 2.84e-6, e22: -1.72e-13, e23: 0.0
  }
};

// Mỗi frame:
setInterval(() => {
  dragToBoneHead(
    camera.position,
    enemy.bones.bone_Head,
    enemy.bindpose,
    kalmanYaw,
    kalmanPitch,
    crosshair,
    "mp40",
"m1887",// có thể là "ak", "m1887", ...
   "ump"
);
}, 16);
