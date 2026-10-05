import * as THREE from "three";
import type { TelemetryStore } from "./telemetry";
import { VehicleMotion } from "./vehicle-motion";

function createBayCheckerTexture(): THREE.DataTexture {
  const size = 8;
  const colors = [
    [216, 59, 59],
    [239, 107, 100],
  ] as const;
  const pixels = new Uint8Array(size * size * 4);
  for (let y = 0; y < size; y += 1) {
    for (let x = 0; x < size; x += 1) {
      const offset = (y * size + x) * 4;
      const color = colors[(x + y) % 2 === 0 ? 0 : 1];
      pixels.set([...color, 255], offset);
    }
  }
  const texture = new THREE.DataTexture(pixels, size, size, THREE.RGBAFormat);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  return texture;
}

export class VehicleScene {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(38, 1, 0.1, 200);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private readonly vehicle = new THREE.Group();
  private readonly rotor = new THREE.Group();
  private readonly targetArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(),
    2,
    0xffd54f,
    0.3,
    0.15,
  );
  private spin = 0;
  private previousFrame = performance.now();
  private animation = 0;
  private dirty = true;
  private observer: ResizeObserver;
  private readonly motion = new VehicleMotion();
  private readonly targetOrigin = new THREE.Vector3();
  private readonly rotorAxis = new THREE.Vector3();
  private readonly previousRotorAxis = new THREE.Vector3(0, 1, 0);
  private readonly unsubscribe: (() => void)[];
  private readonly textures = new Set<THREE.Texture>();

  constructor(
    private readonly container: HTMLElement,
    telemetry: TelemetryStore,
  ) {
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.scene.background = new THREE.Color(0x87ceeb);
    container.append(this.renderer.domElement);

    this.camera.position.set(7, 5, 8);
    this.camera.lookAt(0, 0.7, 0);
    this.scene.add(new THREE.HemisphereLight(0xddeeff, 0x202035, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 2.5);
    key.position.set(4, 8, 6);
    this.scene.add(key);
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(400, 400),
      new THREE.MeshBasicMaterial({ color: 0x579b45, side: THREE.DoubleSide }),
    );
    ground.rotation.x = -Math.PI / 2;
    this.scene.add(ground);
    const grid = new THREE.GridHelper(20, 20, 0x365e2c, 0x487f38);
    grid.position.y = 0.01;
    this.scene.add(grid);

    const targetShaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.04, 0.04, 1.7, 8),
      new THREE.MeshBasicMaterial({ color: 0xffd54f, depthTest: false }),
    );
    targetShaft.position.y = 0.85;
    targetShaft.renderOrder = 10;
    this.targetArrow.add(targetShaft);
    // The target is a visual annotation, including when it points through the ground.
    for (const part of [this.targetArrow.line, this.targetArrow.cone]) {
      const materials = Array.isArray(part.material) ? part.material : [part.material];
      materials.forEach((material) => { material.depthTest = false; });
      part.renderOrder = 10;
    }

    this.buildVehicle();
    this.scene.add(this.vehicle, this.targetArrow);
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(container);
    this.resize();
    this.unsubscribe = [
      telemetry.onRecord((record) => this.motion.accept(record, performance.now())),
      telemetry.onGeneration(() => this.motion.reset()),
      telemetry.onConnection((connected) => {
        if (!connected) {
          this.motion.reset();
        }
      }),
    ];
    for (const message of [
      "ATTITUDE_QUATERNION", "LOCAL_POSITION_NED", "ATTITUDE_TARGET", "ESC_TELEMETRY_1_TO_4",
    ]) {
      const record = telemetry.get(message);
      if (record) {
        this.motion.accept(record, performance.now());
      }
    }
    document.addEventListener("visibilitychange", this.onVisibilityChange);
    this.animate();
  }

  get hasCaptureTarget(): boolean {
    return this.motion.targetVisible;
  }

  dispose(): void {
    cancelAnimationFrame(this.animation);
    this.observer.disconnect();
    document.removeEventListener("visibilitychange", this.onVisibilityChange);
    for (const unsubscribe of this.unsubscribe) {
      unsubscribe();
    }
    const geometries = new Set<THREE.BufferGeometry>();
    const materials = new Set<THREE.Material>();
    this.scene.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.Line) {
        geometries.add(object.geometry);
        if (Array.isArray(object.material)) {
          object.material.forEach((material) => materials.add(material));
        } else {
          materials.add(object.material);
        }
      }
    });
    geometries.forEach((geometry) => geometry.dispose());
    materials.forEach((material) => material.dispose());
    this.textures.forEach((texture) => texture.dispose());
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  private buildVehicle(): void {
    const bayTexture = createBayCheckerTexture();
    this.textures.add(bayTexture);
    const bayMaterial = new THREE.MeshStandardMaterial({
      map: bayTexture,
      roughness: 0.55,
      metalness: 0.25,
    });
    const bay = new THREE.Mesh(
      new THREE.CylinderGeometry(0.55, 0.55, 0.12, 48),
      bayMaterial,
    );
    this.vehicle.add(bay);

    const axle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.055, 0.055, 1.1, 24),
      new THREE.MeshStandardMaterial({ color: 0xb0bec5, metalness: 0.8, roughness: 0.25 }),
    );
    axle.position.y = 0.2;
    this.vehicle.add(axle);

    const forward = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0.08, 0),
      0.8,
      0x08090c,
      0.2,
      0.12,
    );
    this.vehicle.add(forward);

    const hub = new THREE.Mesh(
      new THREE.SphereGeometry(0.13, 24, 16),
      new THREE.MeshStandardMaterial({ color: 0xd7dde2, metalness: 0.8, roughness: 0.2 }),
    );
    this.rotor.add(hub);
    const bladeGeometry = new THREE.BoxGeometry(2.35, 0.025, 0.16);
    bladeGeometry.translate(1.32, 0, 0);
    const bladeMaterials = [0xf5f5f5, 0x263238].map((color) => (
      new THREE.MeshStandardMaterial({ color, roughness: 0.4, metalness: 0.1 })
    ));
    for (let index = 0; index < 4; index += 1) {
      const blade = new THREE.Mesh(
        bladeGeometry,
        bladeMaterials[index % 2],
      );
      blade.rotation.y = index * Math.PI / 2;
      this.rotor.add(blade);
    }
    this.rotor.position.y = 0.35;
    this.vehicle.add(this.rotor);
  }

  private onVisibilityChange = (): void => {
    cancelAnimationFrame(this.animation);
    if (!document.hidden) {
      this.previousFrame = performance.now();
      this.animate();
    }
  };

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.dirty = true;
  }

  private animate = (): void => {
    const now = performance.now();
    const elapsed = Math.min(0.1, (now - this.previousFrame) / 1_000);
    this.previousFrame = now;
    if (this.updateVehicle(elapsed, now) || this.dirty) {
      this.renderer.render(this.scene, this.camera);
      this.dirty = false;
    }
    this.animation = requestAnimationFrame(this.animate);
  };

  private updateVehicle(elapsed: number, now: number): boolean {
    this.motion.update(elapsed, now);
    let changed = false;
    if (1 - Math.abs(this.vehicle.quaternion.dot(this.motion.attitude)) > 1e-10) {
      this.vehicle.quaternion.copy(this.motion.attitude);
      changed = true;
    }
    const positionChanged = this.vehicle.position.distanceToSquared(this.motion.position) > 1e-10;
    if (positionChanged) {
      this.vehicle.position.copy(this.motion.position);
      this.camera.lookAt(this.vehicle.position);
      changed = true;
    }
    this.spin = (this.spin + this.motion.rpm * 2 * Math.PI / 60 * elapsed / 10)
      % (2 * Math.PI);
    if (this.rotor.rotation.y !== this.spin) {
      this.rotor.rotation.y = this.spin;
      changed = true;
    }

    if (this.motion.targetVisible) {
      this.motion.targetArrowPose(this.targetOrigin, this.rotorAxis);
      if (!this.targetArrow.visible
        || this.targetArrow.position.distanceToSquared(this.targetOrigin) > 1e-10
        || this.rotorAxis.distanceToSquared(this.previousRotorAxis) > 1e-10) {
        this.previousRotorAxis.copy(this.rotorAxis);
        this.targetArrow.position.copy(this.targetOrigin);
        this.targetArrow.setDirection(this.rotorAxis);
        changed = true;
      }
    }
    if (this.targetArrow.visible !== this.motion.targetVisible) {
      this.targetArrow.visible = this.motion.targetVisible;
      changed = true;
    }
    return changed;
  }
}
