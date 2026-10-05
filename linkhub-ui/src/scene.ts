import * as THREE from "three";
import type { TelemetryStore } from "./telemetry";
import type { MessageRecord, Quaternion } from "./types";

const worldFromNed = new THREE.Matrix3().set(
  0, 1, 0,
  0, 0, -1,
  -1, 0, 0,
);
const bodyFromModel = new THREE.Matrix3().set(
  1, 0, 0,
  0, 0, -1,
  0, 1, 0,
);

function numberField(record: MessageRecord | undefined, name: string): number | null {
  const value = record?.fields[name];
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function quaternion(record: MessageRecord | undefined): Quaternion | null {
  if (!record) {
    return null;
  }
  const q1 = numberField(record, "q1");
  const q2 = numberField(record, "q2");
  const q3 = numberField(record, "q3");
  const q4 = numberField(record, "q4");
  if (q1 === null || q2 === null || q3 === null || q4 === null) {
    return null;
  }
  return [q1, q2, q3, q4];
}

function rotationFromQuaternion([w, x, y, z]: Quaternion): THREE.Matrix3 {
  return new THREE.Matrix3().set(
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  );
}

export class VehicleScene {
  private readonly scene = new THREE.Scene();
  private readonly camera = new THREE.PerspectiveCamera(38, 1, 0.1, 200);
  private readonly renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  private readonly vehicle = new THREE.Group();
  private readonly rotor = new THREE.Group();
  private readonly targetArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(),
    2,
    0xffd54f,
    0.3,
    0.15,
  );
  private spin = 0;
  private previousFrame = performance.now();
  private animation = 0;
  private observer: ResizeObserver;

  constructor(
    private readonly container: HTMLElement,
    private readonly telemetry: TelemetryStore,
  ) {
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.append(this.renderer.domElement);

    this.camera.position.set(7, 5, 8);
    this.camera.lookAt(0, 0.7, 0);
    this.scene.add(new THREE.HemisphereLight(0xddeeff, 0x202035, 2.2));
    const key = new THREE.DirectionalLight(0xffffff, 2.5);
    key.position.set(4, 8, 6);
    this.scene.add(key);
    this.scene.add(new THREE.GridHelper(20, 20, 0x2d4160, 0x1b263c));

    this.buildVehicle();
    this.scene.add(this.vehicle, this.targetArrow);
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(container);
    this.resize();
    this.animate();
  }

  dispose(): void {
    cancelAnimationFrame(this.animation);
    this.observer.disconnect();
    this.renderer.dispose();
  }

  private buildVehicle(): void {
    const bayMaterial = new THREE.MeshStandardMaterial({
      color: 0xd83b3b,
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
    const colors = [0xf5f5f5, 0x263238, 0xf5f5f5, 0x263238];
    for (let index = 0; index < 4; index += 1) {
      const blade = new THREE.Mesh(
        bladeGeometry,
        new THREE.MeshStandardMaterial({
          color: colors[index],
          roughness: 0.4,
          metalness: 0.1,
        }),
      );
      blade.rotation.y = index * Math.PI / 2;
      this.rotor.add(blade);
    }
    this.rotor.position.y = 0.35;
    this.vehicle.add(this.rotor);
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  private animate = (): void => {
    const now = performance.now();
    const elapsed = Math.min(0.1, (now - this.previousFrame) / 1_000);
    this.previousFrame = now;
    this.updateVehicle(elapsed);
    this.renderer.render(this.scene, this.camera);
    this.animation = requestAnimationFrame(this.animate);
  };

  private updateVehicle(elapsed: number): void {
    const attitude = this.telemetry.get("ATTITUDE_QUATERNION");
    const actual = quaternion(attitude);
    if (actual) {
      const rotation = worldFromNed.clone()
        .multiply(rotationFromQuaternion(actual))
        .multiply(bodyFromModel);
      const matrix = new THREE.Matrix4().setFromMatrix3(rotation);
      this.vehicle.quaternion.setFromRotationMatrix(matrix);
    }

    const position = this.telemetry.get("LOCAL_POSITION_NED");
    const north = numberField(position, "x") ?? 0;
    const east = numberField(position, "y") ?? 0;
    const down = numberField(position, "z") ?? -5;
    this.vehicle.position.set(east, -down, -north);

    const rpm = numberField(this.telemetry.get("ESC_TELEMETRY_1_TO_4"), "rpm1") ?? 0;
    this.spin += rpm * 2 * Math.PI / 60 * elapsed / 10;
    this.rotor.rotation.y = this.spin;

    const target = quaternion(this.telemetry.get("ATTITUDE_TARGET"));
    if (target) {
      const bodyDown = new THREE.Vector3(0, 0, 1)
        .applyMatrix3(rotationFromQuaternion(target))
        .applyMatrix3(worldFromNed)
        .normalize();
      this.targetArrow.position.copy(this.vehicle.position);
      this.targetArrow.setDirection(bodyDown);
      this.targetArrow.visible = true;
    } else {
      this.targetArrow.visible = false;
    }
    this.camera.lookAt(this.vehicle.position);
  }
}
