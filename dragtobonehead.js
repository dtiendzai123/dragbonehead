
// === Enhanced Vector3 với Performance Optimization ===
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { 
    this.x = x; this.y = y; this.z = z; 
  }

  add(v) { return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z); }
  subtract(v) { return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z); }
  multiplyScalar(s) { return new Vector3(this.x * s, this.y * s, this.z * s); }
  multiply(v) { return new Vector3(this.x * v.x, this.y * v.y, this.z * v.z); }
  divide(v) { return new Vector3(this.x / v.x, this.y / v.y, this.z / v.z); }
  dot(v) { return this.x * v.x + this.y * v.y + this.z * v.z; }
  cross(v) { 
    return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x
    ); 
  }

  clone() { return new Vector3(this.x, this.y, this.z); }
  length() { return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z); }
  lengthSq() { return this.x * this.x + this.y * this.y + this.z * this.z; }
  
  normalize() {
    const len = this.length();
    return len === 0 ? this.clone() : this.multiplyScalar(1 / len);
  }

  distanceTo(v) { return this.subtract(v).length(); }
  distanceToSq(v) { return this.subtract(v).lengthSq(); }

  lerp(v, t) {
    return new Vector3(
      this.x + (v.x - this.x) * t,
      this.y + (v.y - this.y) * t,
      this.z + (v.z - this.z) * t
    );
  }

  slerp(v, t) {
    const dot = Math.max(-1, Math.min(1, this.normalize().dot(v.normalize())));
    const theta = Math.acos(dot) * t;
    const relative = v.subtract(this.multiplyScalar(dot)).normalize();
    return this.multiplyScalar(Math.cos(theta)).add(relative.multiplyScalar(Math.sin(theta)));
  }

  static zero() { return new Vector3(0, 0, 0); }
  static one() { return new Vector3(1, 1, 1); }
  static forward() { return new Vector3(0, 0, 1); }
  static up() { return new Vector3(0, 1, 0); }
  static right() { return new Vector3(1, 0, 0); }
}

// === Quaternion Class cho Rotation tối ưu ===
class Quaternion {
  constructor(x = 0, y = 0, z = 0, w = 1) {
    this.x = x; this.y = y; this.z = z; this.w = w;
  }

  static fromEuler(pitch, yaw, roll) {
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);

    return new Quaternion(
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy
    );
  }

  multiply(q) {
    return new Quaternion(
      this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
      this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
      this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w,
      this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z
    );
  }

  rotateVector(v) {
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    const x = v.x, y = v.y, z = v.z;

    const ix = qw * x + qy * z - qz * y;
    const iy = qw * y + qz * x - qx * z;
    const iz = qw * z + qx * y - qy * x;
    const iw = -qx * x - qy * y - qz * z;

    return new Vector3(
      ix * qw + iw * -qx + iy * -qz - iz * -qy,
      iy * qw + iw * -qy + iz * -qx - ix * -qz,
      iz * qw + iw * -qz + ix * -qy - iy * -qx
    );
  }
}

// === Enhanced Transform System ===
function applyFullTransform(bone, bindpose) {
  const { position, rotation, scale } = bone;
  
  // Scale transformation
  let scaledPos = new Vector3(
    position.x * scale.x,
    position.y * scale.y,
    position.z * scale.z
  );

  // Quaternion rotation
  const quat = new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  const rotatedPos = quat.rotateVector(scaledPos);

  // Matrix transformation
  const { e00, e01, e02, e03, e10, e11, e12, e13, e20, e21, e22, e23 } = bindpose;
  
  return new Vector3(
    e00 * rotatedPos.x + e01 * rotatedPos.y + e02 * rotatedPos.z + e03,
    e10 * rotatedPos.x + e11 * rotatedPos.y + e12 * rotatedPos.z + e13,
    e20 * rotatedPos.x + e21 * rotatedPos.y + e22 * rotatedPos.z + e23
  );
}

// === Advanced Kalman Filter với Adaptive Learning ===
class AdaptiveKalmanFilter {
  constructor(r = 0.01, q = 0.1, adaptiveRate = 0.05) {
    this.r = r; // Measurement noise
    this.q = q; // Process noise
    this.p = 1; // Estimation error
    this.x = 0; // State
    this.adaptiveRate = adaptiveRate;
    this.innovation = 0;
    this.innovationHistory = [];
    this.maxHistory = 20;
  }

  update(measurement) {
    // Prediction step
    this.p += this.q;
    
    // Innovation (residual)
    this.innovation = measurement - this.x;
    this.innovationHistory.push(Math.abs(this.innovation));
    
    if (this.innovationHistory.length > this.maxHistory) {
      this.innovationHistory.shift();
    }

    // Adaptive noise adjustment
    if (this.innovationHistory.length >= 5) {
      const avgInnovation = this.innovationHistory.reduce((a, b) => a + b, 0) / this.innovationHistory.length;
      const adaptiveFactor = Math.max(0.1, Math.min(2.0, avgInnovation * 10));
      this.q = Math.max(0.01, Math.min(0.5, this.q + (adaptiveFactor - 1) * this.adaptiveRate));
    }

    // Update step
    const k = this.p / (this.p + this.r);
    this.x += k * this.innovation;
    this.p *= (1 - k);
    
    return this.x;
  }

  reset() {
    this.p = 1;
    this.x = 0;
    this.innovationHistory = [];
  }
}

// === Velocity Prediction System ===
class VelocityPredictor {
  constructor(historySize = 10) {
    this.positionHistory = [];
    this.velocityHistory = [];
    this.accelerationHistory = [];
    this.historySize = historySize;
    this.lastTime = performance.now();
  }

  addPosition(position) {
    const currentTime = performance.now();
    const deltaTime = (currentTime - this.lastTime) / 1000; // Convert to seconds
    
    if (deltaTime <= 0) return Vector3.zero();

    this.positionHistory.push({ pos: position.clone(), time: currentTime });
    
    if (this.positionHistory.length > this.historySize) {
      this.positionHistory.shift();
    }

    // Calculate velocity
    if (this.positionHistory.length >= 2) {
      const current = this.positionHistory[this.positionHistory.length - 1];
      const previous = this.positionHistory[this.positionHistory.length - 2];
      const timeDiff = (current.time - previous.time) / 1000;
      
      if (timeDiff > 0) {
        const velocity = current.pos.subtract(previous.pos).multiplyScalar(1 / timeDiff);
        this.velocityHistory.push(velocity);
        
        if (this.velocityHistory.length > this.historySize) {
          this.velocityHistory.shift();
        }
      }
    }

    // Calculate acceleration
    if (this.velocityHistory.length >= 2) {
      const currentVel = this.velocityHistory[this.velocityHistory.length - 1];
      const previousVel = this.velocityHistory[this.velocityHistory.length - 2];
      const acceleration = currentVel.subtract(previousVel).multiplyScalar(1 / deltaTime);
      
      this.accelerationHistory.push(acceleration);
      if (this.accelerationHistory.length > this.historySize) {
        this.accelerationHistory.shift();
      }
    }

    this.lastTime = currentTime;
    return this.getCurrentVelocity();
  }

  getCurrentVelocity() {
    if (this.velocityHistory.length === 0) return Vector3.zero();
    
    // Average recent velocities for smoothing
    const recentCount = Math.min(3, this.velocityHistory.length);
    const recentVelocities = this.velocityHistory.slice(-recentCount);
    
    const avgVelocity = recentVelocities.reduce((sum, vel) => sum.add(vel), Vector3.zero())
                                       .multiplyScalar(1 / recentCount);
    return avgVelocity;
  }

  predictPosition(currentPos, deltaTime) {
    const velocity = this.getCurrentVelocity();
    if (velocity.length() < 0.1) return currentPos; // Static target
    
    const predictedPos = currentPos.add(velocity.multiplyScalar(deltaTime));
    
    // Add acceleration if available
    if (this.accelerationHistory.length > 0) {
      const acceleration = this.accelerationHistory[this.accelerationHistory.length - 1];
      const accelContribution = acceleration.multiplyScalar(0.5 * deltaTime * deltaTime);
      return predictedPos.add(accelContribution);
    }
    
    return predictedPos;
  }
}

// === Advanced Weapon Configuration ===
const weaponDragConfig = {
  // Assault Rifles
  ak: { 
    smooth: 0.0012, snapRadius: 380, strongPullRadius: 450, maxSpeed: 800,
    sensitivity: { x: 0.0012, y: 0.0008 }, recoilPattern: [0.1, 0.15, 0.2, 0.18, 0.12],
    fireRate: 600, range: 85, stability: 0.7
  },
  m4a1: { 
    smooth: 0.0015, snapRadius: 400, strongPullRadius: 480, maxSpeed: 750,
    sensitivity: { x: 0.0015, y: 0.001 }, recoilPattern: [0.08, 0.12, 0.16, 0.14, 0.1],
    fireRate: 650, range: 90, stability: 0.8
  },
  
  // SMGs
  mp40: { 
    smooth: 0.0008, snapRadius: 340, strongPullRadius: 400, maxSpeed: 900,
    sensitivity: { x: 0.0008, y: 0.0006 }, recoilPattern: [0.06, 0.09, 0.12, 0.1, 0.08],
    fireRate: 750, range: 60, stability: 0.85
  },
  ump: { 
    smooth: 0.001, snapRadius: 360, strongPullRadius: 420, maxSpeed: 850,
    sensitivity: { x: 0.001, y: 0.0007 }, recoilPattern: [0.07, 0.11, 0.14, 0.12, 0.09],
    fireRate: 700,  range: 65, stability: 0.82
  },
  
  // Shotguns
  m1887: { 
    smooth: 0.002, snapRadius: 450, strongPullRadius: 550, maxSpeed: 600,
    sensitivity: { x: 0.002, y: 0.0015 }, recoilPattern: [0.3, 0.25, 0.2],
    fireRate: 120, range: 25, stability: 0.6
  },
  
  // Sniper Rifles
  awm: { 
    smooth: 0.003, snapRadius: 500, strongPullRadius: 600, maxSpeed: 400,
    sensitivity: { x: 0.003, y: 0.002 }, recoilPattern: [0.4],
    fireRate: 60, range: 100, stability: 0.95
  },
  
  // Default fallback
  default: { 
    smooth: 0.001, snapRadius: 360, strongPullRadius: 400, maxSpeed: 750,
    sensitivity: { x: 0.001, y: 0.0008 }, recoilPattern: [0.1, 0.12, 0.15, 0.13, 0.1],
    fireRate: 600, range: 75, stability: 0.75
  }
};

// === Anti-Detection System ===
class AntiDetectionSystem {
  constructor() {
    this.humanization = {
      reactionTime: { min: 120, max: 280 }, // ms
      mouseShake: { intensity: 0.02, frequency: 0.3 },
      aimDrift: { intensity: 0.01, frequency: 0.1 },
      missChance: 0.05, // 5% intentional miss
      pauseChance: 0.02 // 2% chance to pause briefly
    };
    
    this.adaptiveSettings = {
      aggressiveness: 0.7, // 0-1 scale
      detectionLevel: 0, // Increases with suspicious activity
      maxDetectionLevel: 100
    };
    
    this.behaviorPatterns = [];
    this.lastAction = performance.now();
  }

  addHumanization(deltaX, deltaY) {
    const now = performance.now();
    
    // Add reaction time delay
    const reactionDelay = this.humanization.reactionTime.min + 
                         Math.random() * (this.humanization.reactionTime.max - this.humanization.reactionTime.min);
    
    // Add mouse shake
    const shakeX = (Math.random() - 0.5) * this.humanization.mouseShake.intensity;
    const shakeY = (Math.random() - 0.5) * this.humanization.mouseShake.intensity;
    
    // Add subtle aim drift
    const driftX = Math.sin(now * this.humanization.aimDrift.frequency) * this.humanization.aimDrift.intensity;
    const driftY = Math.cos(now * this.humanization.aimDrift.frequency) * this.humanization.aimDrift.intensity;
    
    // Random miss chance
    if (Math.random() < this.humanization.missChance) {
      const missMultiplier = 1 + (Math.random() - 0.5) * 0.3;
      deltaX *= missMultiplier;
      deltaY *= missMultiplier;
    }
    
    // Random pause
    if (Math.random() < this.humanization.pauseChance) {
      setTimeout(() => {
        this.sendDelayedInput(deltaX + shakeX + driftX, deltaY + shakeY + driftY);
      }, reactionDelay);
      return;
    }
    
    return {
      deltaX: deltaX + shakeX + driftX,
      deltaY: deltaY + shakeY + driftY,
      delay: reactionDelay
    };
  }

  updateDetectionLevel(suspiciousActivity) {
    this.adaptiveSettings.detectionLevel += suspiciousActivity;
    this.adaptiveSettings.detectionLevel = Math.max(0, 
      Math.min(this.adaptiveSettings.maxDetectionLevel, this.adaptiveSettings.detectionLevel));
    
    // Adjust aggressiveness based on detection level
    const detectionRatio = this.adaptiveSettings.detectionLevel / this.adaptiveSettings.maxDetectionLevel;
    this.adaptiveSettings.aggressiveness = Math.max(0.3, 1.0 - detectionRatio * 0.5);
  }

  sendDelayedInput(deltaX, deltaY) {
    sendInputToMouse({ deltaX, deltaY });
  }
}

// === Performance Monitor ===
class PerformanceMonitor {
  constructor() {
    this.frameCount = 0;
    this.lastFPSUpdate = performance.now();
    this.fps = 0;
    this.avgFrameTime = 0;
    this.frameTimes = [];
    this.maxFrameTimes = 60;
    
    this.stats = {
      totalFrames: 0,
      totalAimTime: 0,
      successfulHits: 0,
      totalShots: 0,
      accuracy: 0
    };
  }

  startFrame() {
    this.frameStart = performance.now();
  }

  endFrame() {
    const frameTime = performance.now() - this.frameStart;
    this.frameTimes.push(frameTime);
    
    if (this.frameTimes.length > this.maxFrameTimes) {
      this.frameTimes.shift();
    }
    
    this.avgFrameTime = this.frameTimes.reduce((a, b) => a + b, 0) / this.frameTimes.length;
    
    this.frameCount++;
    this.stats.totalFrames++;
    
    const now = performance.now();
    if (now - this.lastFPSUpdate >= 1000) {
      this.fps = this.frameCount;
      this.frameCount = 0;
      this.lastFPSUpdate = now;
    }
  }

  getStats() {
    return {
      fps: this.fps,
      avgFrameTime: this.avgFrameTime.toFixed(2) + 'ms',
      accuracy: this.stats.totalShots > 0 ? 
                ((this.stats.successfulHits / this.stats.totalShots) * 100).toFixed(1) + '%' : '0%',
      totalFrames: this.stats.totalFrames,
      uptime: ((performance.now() - this.stats.startTime) / 1000).toFixed(1) + 's'
    };
  }
}

// === Enhanced Input System ===
function sendInputToMouse({ deltaX, deltaY, smooth = true, humanize = true }) {
  if (humanize && antiDetection) {
    const humanized = antiDetection.addHumanization(deltaX, deltaY);
    if (humanized) {
      setTimeout(() => {
        console.log(`🎯 Enhanced Aim → ΔX=${humanized.deltaX.toFixed(6)} | ΔY=${humanized.deltaY.toFixed(6)} | Delay=${humanized.delay}ms`);
        // Send to actual mouse input system here
      }, humanized.delay);
      return;
    }
  }
  
  if (smooth) {
    const smoothFactor = 0.8 + Math.random() * 0.4; // 0.8-1.2 randomization
    deltaX *= smoothFactor;
    deltaY *= smoothFactor;
  }
  
  console.log(`🎯 Enhanced Aim → ΔX=${deltaX.toFixed(6)} | ΔY=${deltaY.toFixed(6)}`);
  // Send to actual mouse input system here
}

// === Game Package Detection ===
const GamePackages = {
  GamePackage1: "com.dts.freefireth",
  GamePackage2: "com.dts.freefiremax",
 
};

// === Main Enhanced Aimbot System ===
function enhancedDragToBoneHead(
  cameraPos, boneHead, bindpose,
  kalmanYaw, kalmanPitch,
  crosshair, weaponType = "default",
  options = {}
) {
  // Performance monitoring
  perfMonitor.startFrame();
  
  const headWorld = applyFullTransform(boneHead, bindpose);
  
  // Update velocity prediction
  const velocity = velocityPredictor.addPosition(headWorld);
  const predictionTime = options.predictionTime || 0.15; // 150ms ahead
  const predictedHeadPos = velocityPredictor.predictPosition(headWorld, predictionTime);
  
  // Use predicted position if target is moving
  const targetPos = velocity.length() > 0.5 ? predictedHeadPos : headWorld;
  
  const dir = targetPos.subtract(new Vector3(cameraPos.x, cameraPos.y, cameraPos.z)).normalize();

  // Calculate aim angles
  const pitch = -Math.asin(Math.max(-1, Math.min(1, dir.y)));
  const yaw = Math.atan2(dir.x, dir.z);

  // Enhanced Kalman filtering with adaptive learning
  const smoothYaw = kalmanYaw.update(yaw);
  const smoothPitch = kalmanPitch.update(pitch);

  // Get weapon configuration
  const config = weaponDragConfig[weaponType] || weaponDragConfig["default"];
  
  // Apply anti-detection scaling
  const detectionMultiplier = antiDetection.adaptiveSettings.aggressiveness;
  const sensitivity = {
    x: config.sensitivity.x * detectionMultiplier,
    y: config.sensitivity.y * detectionMultiplier
  };
  
  const maxPullSpeed = config.maxSpeed * detectionMultiplier;
  const dragSmooth = config.smooth;
  const snapRadius = config.snapRadius;
  const strongPullRadius = config.strongPullRadius;
  const maxRadius = config.maxRadius || 360.0;

  // Calculate crosshair offset to target
  const dx = targetPos.x - crosshair.x;
  const dy = targetPos.y - crosshair.y;
  const dist = Math.hypot(dx, dy);

  // Instant snap for very close targets
  if (dist < snapRadius * 0.3) {
    crosshair.x = targetPos.x;
    crosshair.y = targetPos.y;
    
    perfMonitor.endFrame();
    return sendInputToMouse({ 
      deltaX: dx, 
      deltaY: dy, 
      smooth: false,
      humanize: options.humanize !== false 
    });
  }

  // Gradual snap for close targets
  if (dist < snapRadius) {
    const snapIntensity = 1 - (dist / snapRadius);
    const snapDx = dx * (0.3 + snapIntensity * 0.7);
    const snapDy = dy * (0.3 + snapIntensity * 0.7);
    
    crosshair.x += snapDx;
    crosshair.y += snapDy;
    
    perfMonitor.endFrame();
    return sendInputToMouse({ 
      deltaX: snapDx, 
      deltaY: snapDy,
      humanize: options.humanize !== false 
    });
  }

  // Enhanced easing calculation
  let ease;
  const baseEase = Math.min(1, dragSmooth * (dist / maxPullSpeed));
  
  if (dist < strongPullRadius) {
    // Stronger pull in close range
    const proximityFactor = 1 + Math.pow((strongPullRadius - dist) / strongPullRadius, 2) * 2;
    ease = Math.min(1, baseEase * proximityFactor);
  } else {
    // Gentle pull for distant targets
    const distanceFactor = Math.max(0.1, 1 - (dist - strongPullRadius) / (maxRadius - strongPullRadius));
    ease = baseEase * distanceFactor;
  }

  // Apply movement prediction compensation
  if (velocity.length() > 0.1) {
    const velocityCompensation = velocity.multiplyScalar(predictionTime * 0.5);
    ease *= (1 + velocityCompensation.length() * 0.1);
  }

  // Calculate new crosshair position
  let newX = crosshair.x + dx * ease * sensitivity.x;
  let newY = crosshair.y + dy * ease * sensitivity.y;

  // Boundary constraints
  const offsetX = newX - targetPos.x;
  const offsetY = newY - targetPos.y;
  const offsetDist = Math.hypot(offsetX, offsetY);

  if (offsetDist > maxRadius) {
    const clampFactor = maxRadius / offsetDist;
    newX = targetPos.x + offsetX * clampFactor;
    newY = targetPos.y + offsetY * clampFactor;
  }

  // Calculate final deltas
  const deltaX = newX - crosshair.x;
  const deltaY = newY - crosshair.y;

  // Update crosshair position
  crosshair.x = newX;
  crosshair.y = newY;

  // Update detection level based on movement magnitude
  const movementMagnitude = Math.hypot(deltaX, deltaY);
  antiDetection.updateDetectionLevel(movementMagnitude > 10 ? 1 : -0.1);

  perfMonitor.endFrame();
  
  sendInputToMouse({ 
    deltaX, 
    deltaY, 
    humanize: options.humanize !== false 
  });
}

// === Initialize Systems ===
const kalmanYaw = new AdaptiveKalmanFilter(0.003, 0.008, 0.02);
const kalmanPitch = new AdaptiveKalmanFilter(0.003, 0.008, 0.02);
const velocityPredictor = new VelocityPredictor(15);
const antiDetection = new AntiDetectionSystem();
const perfMonitor = new PerformanceMonitor();

// === Game State ===
const camera = { position: { x: 0, y: 1.7, z: 0 } };
const crosshair = { x: 0, y: 0 };

// === Multi-Target System ===
const enemies = new Map();

// Add enemy example
const enemy1 = {
  id: "enemy_1",
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
  },
  threat: 0.8, // Threat level 0-1
  distance: 9999,
  isVisible: true
};

enemies.set(enemy1.id, enemy1);

// === Target Selection AI ===

function selectBestTarget() {
  const visibleEnemies = Array.from(enemies.values()).filter(enemy => enemy.isVisible);
  
  if (visibleEnemies.length === 0) return null;
  if (visibleEnemies.length === 1) return visibleEnemies[0];
  
  // Multi-factor scoring
  let bestEnemy = null;
  let bestScore = -1;
  
  for (const enemy of visibleEnemies) {
    let score = 0;
    
    // Distance factor (closer is better)
    score += Math.max(0, 100 - enemy.distance) * 0.3;
    
    // Threat factor
    score += enemy.threat * 40;
    
    // Crosshair proximity
    const headWorld = applyFullTransform(enemy.bones.bone_Head, enemy.bindpose);
    const crosshairDist = Math.hypot(crosshair.x - headWorld.x, crosshair.y - headWorld.y);
    score += Math.max(0, 50 - crosshairDist) * 0.2;
    
    // Health factor (if available)
    if (enemy.health !== undefined) {
      score += (1 - enemy.health) * 20; // Lower health = higher priority
    }
    
    if (score > bestScore) {
      bestScore = score;
      bestEnemy = enemy;
    }
  }

  return bestEnemy;
}
