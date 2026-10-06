import * as THREE from "three";
import { describe, expect, it } from "vitest";
import { buildAirframe, ROTOR } from "../src/airframe";

describe("buildAirframe", () => {
  it("keeps the rotor hub 0.35 m above the bay at the vehicle origin", () => {
    const { root, rotor } = buildAirframe();
    expect(root.position.toArray()).toEqual([0, 0, 0]);
    expect(rotor.position.toArray()).toEqual([0, 0.35, 0]);
    expect(rotor.parent).toBe(root);
  });

  it("spans the rotor definition radius with one arm per blade", () => {
    const { rotor } = buildAirframe();
    const arms = rotor.children.filter((child) => child instanceof THREE.Group);
    expect(arms).toHaveLength(ROTOR.bladeCount);
    const bounds = new THREE.Box3().setFromObject(rotor);
    const reach = Math.max(bounds.max.x, bounds.max.z, -bounds.min.x, -bounds.min.z);
    expect(reach).toBeCloseTo(ROTOR.radiusM, 2);
    expect(bounds.max.y - bounds.min.y).toBeLessThan(0.2);
  });

  it("spins only the rotor, leaving the instrument bay stationary", () => {
    const { root, rotor } = buildAirframe();
    const before = new THREE.Box3().setFromObject(root);
    rotor.rotation.y = 0.7;
    rotor.updateMatrixWorld(true);
    const stationary = root.children.filter((child) => child !== rotor);
    expect(stationary.length).toBeGreaterThan(0);
    for (const part of stationary) {
      expect(part.rotation.y).toBe(0);
    }
    const after = new THREE.Box3().setFromObject(root);
    expect(after.min.y).toBeCloseTo(before.min.y, 6);
    expect(after.max.y).toBeCloseTo(before.max.y, 6);
  });

  it("marks one blade with a black dot on each face, inside the blade outline", () => {
    const { rotor } = buildAirframe();
    const marks = rotor.getObjectByName("blade-mark-top");
    const underside = rotor.getObjectByName("blade-mark-bottom");
    expect(marks).toBeDefined();
    expect(underside).toBeDefined();
    let count = 0;
    rotor.traverse((object) => { if (object.name.startsWith("blade-mark")) count += 1; });
    expect(count).toBe(2);
    for (const mark of [marks, underside]) {
      const bounds = new THREE.Box3().setFromObject(mark as THREE.Object3D);
      expect(bounds.min.x).toBeGreaterThan(ROTOR.rootCutoutM);
      expect(bounds.max.x).toBeLessThan(ROTOR.radiusM);
      expect(Math.max(bounds.max.z, -bounds.min.z)).toBeLessThan(ROTOR.chordM / 2);
    }
    expect((marks as THREE.Object3D).position.y).toBeGreaterThan(0);
    expect((underside as THREE.Object3D).position.y).toBeLessThan(0);
  });

  it("returns the bay texture so the scene can dispose it", () => {
    const { textures } = buildAirframe();
    expect(textures).toHaveLength(1);
    expect(textures[0]).toBeInstanceOf(THREE.DataTexture);
  });
});
