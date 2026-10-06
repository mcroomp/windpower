import * as THREE from "three";

// Planform from simulation/rotor_definitions/beaupoil_2026.yaml. The remaining
// dimensions are visual assumptions shared with viz3d/visualize_rotor_model.py.
// The rotor axis is +Y here (the Python model's +Z).
export const ROTOR = {
  bladeCount: 4,
  radiusM: 2.5,
  rootCutoutM: 0.5,
  chordM: 0.2,
} as const;

const SIZE = {
  rotorHubRadius: 0.15,
  rotorHubThickness: 0.08,
  instrumentHubRadius: 0.15,
  instrumentHubThickness: 0.08,
  axleRadius: 0.0125,
  bladeThickness: 0.0225,
  antennaLength: 0.09,
  antennaRadius: 0.008,
  housingLength: 0.18,
  housingWidth: 0.14,
  housingHeight: 0.12,
  structureRodRadius: 0.014,
  bladeLinkLength: 0.13,
  bladeLinkEmbed: 0.02,
  tetherLength: 0.8,
  tetherRadius: 0.012,
  markRadius: ROTOR.chordM * 0.32,
  markInsetFromTip: 0.3,
  markLift: 0.0005,
} as const;

const COLOR = {
  wood: 0xd7a85d,
  woodEdge: 0x8b5a2b,
  rotorHub: 0x2f6f9f,
  axle: 0xc4cbd0,
  housing: 0xf1f0e8,
  structure: 0x151718,
  antenna: 0x111315,
  tether: 0x26282a,
  mark: 0x111315,
} as const;

// Layout is unchanged from the previous scene: the vehicle origin is the
// instrument bay, the rotor hub sits above it, and VehicleMotion.targetArrowPose
// is anchored relative to that origin.
const ROTOR_HUB_HEIGHT_M = 0.35;
const AXLE_BOTTOM_M = -0.35;
const AXLE_TOP_M = 0.75;
const FORWARD_ARROW_HEIGHT_M = 0.08;

export interface Airframe {
  root: THREE.Group;
  rotor: THREE.Group;
  textures: THREE.Texture[];
}

function createInstrumentHubTexture(): THREE.DataTexture {
  const size = 8;
  const colors = [
    [216, 59, 59],
    [239, 107, 100],
  ] as const;
  const pixels = new Uint8Array(size * size * 4);
  for (let y = 0; y < size; y += 1) {
    for (let x = 0; x < size; x += 1) {
      const color = colors[(x + y) % 2 === 0 ? 0 : 1];
      pixels.set([...color, 255], (y * size + x) * 4);
    }
  }
  const texture = new THREE.DataTexture(pixels, size, size, THREE.RGBAFormat);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  return texture;
}

// Chamfer-tipped planform in the X-Z plane, extending along +X.
function createBladeGeometry(rootRadius: number, tipRadius: number): THREE.BufferGeometry {
  const halfChord = ROTOR.chordM / 2;
  const chamfer = Math.min(ROTOR.chordM * 0.22, (tipRadius - rootRadius) * 0.08);
  const outline = new THREE.Shape([
    new THREE.Vector2(rootRadius, -halfChord),
    new THREE.Vector2(tipRadius - chamfer, -halfChord),
    new THREE.Vector2(tipRadius, -halfChord + chamfer),
    new THREE.Vector2(tipRadius, halfChord - chamfer),
    new THREE.Vector2(tipRadius - chamfer, halfChord),
    new THREE.Vector2(rootRadius, halfChord),
  ]);
  const geometry = new THREE.ExtrudeGeometry(outline, {
    depth: SIZE.bladeThickness,
    bevelEnabled: false,
  });
  geometry.translate(0, 0, -SIZE.bladeThickness / 2);
  geometry.rotateX(-Math.PI / 2);
  return geometry;
}

function createRod(
  from: THREE.Vector3,
  to: THREE.Vector3,
  radius: number,
  material: THREE.Material,
): THREE.Mesh {
  const direction = to.clone().sub(from);
  const rod = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, direction.length(), 16),
    material,
  );
  rod.position.copy(from).add(to).multiplyScalar(0.5);
  rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return rod;
}

function createDisc(
  radius: number,
  thickness: number,
  y: number,
  material: THREE.Material,
): THREE.Mesh {
  const disc = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, thickness, 72), material);
  disc.position.y = y;
  return disc;
}

export function buildAirframe(): Airframe {
  const root = new THREE.Group();
  const rotor = new THREE.Group();
  rotor.position.y = ROTOR_HUB_HEIGHT_M;
  const texture = createInstrumentHubTexture();

  const wood = new THREE.MeshStandardMaterial({ color: COLOR.wood, roughness: 0.72, metalness: 0 });
  const woodEdge = new THREE.LineBasicMaterial({ color: COLOR.woodEdge });
  const structure = new THREE.MeshStandardMaterial({
    color: COLOR.structure, roughness: 0.65, metalness: 0.15,
  });
  const housing = new THREE.MeshStandardMaterial({
    color: COLOR.housing, roughness: 0.75, metalness: 0,
  });
  const markGeometry = new THREE.CircleGeometry(SIZE.markRadius, 32);
  const markMaterial = new THREE.MeshStandardMaterial({
    color: COLOR.mark,
    roughness: 0.6,
    metalness: 0,
    polygonOffset: true,
    polygonOffsetFactor: -2,
    polygonOffsetUnits: -2,
  });

  const housingRadius = ROTOR.rootCutoutM + 0.035;
  const housingOuterRadius = housingRadius + SIZE.housingLength / 2;
  const bladeRoot = housingOuterRadius + SIZE.bladeLinkLength - SIZE.bladeLinkEmbed;
  const bladeGeometry = createBladeGeometry(bladeRoot, ROTOR.radiusM);
  const bladeEdges = new THREE.EdgesGeometry(bladeGeometry);
  const housingGeometry = new THREE.BoxGeometry(
    SIZE.housingLength, SIZE.housingHeight, SIZE.housingWidth,
  );
  const housingCenters: THREE.Vector3[] = [];

  rotor.add(createDisc(
    SIZE.rotorHubRadius,
    SIZE.rotorHubThickness,
    0,
    new THREE.MeshStandardMaterial({ color: COLOR.rotorHub, roughness: 0.45, metalness: 0.15 }),
  ));

  for (let index = 0; index < ROTOR.bladeCount; index += 1) {
    const angle = index * 2 * Math.PI / ROTOR.bladeCount;
    const arm = new THREE.Group();
    arm.rotation.y = angle;

    arm.add(new THREE.Mesh(bladeGeometry, wood), new THREE.LineSegments(bladeEdges, woodEdge));

    const housingMesh = new THREE.Mesh(housingGeometry, housing);
    housingMesh.position.x = housingRadius;
    arm.add(housingMesh);

    arm.add(createRod(
      new THREE.Vector3(SIZE.rotorHubRadius, 0, 0),
      new THREE.Vector3(housingRadius - SIZE.housingLength / 2, 0, 0),
      SIZE.structureRodRadius,
      structure,
    ));
    arm.add(createRod(
      new THREE.Vector3(housingOuterRadius, 0, 0),
      new THREE.Vector3(housingOuterRadius + SIZE.bladeLinkLength, 0, 0),
      SIZE.structureRodRadius,
      structure,
    ));

    // A black dot on both faces of one blade breaks the four-fold symmetry so
    // rotor spin is visible.
    if (index === 0) {
      for (const [side, name] of [[1, "blade-mark-top"], [-1, "blade-mark-bottom"]] as const) {
        const mark = new THREE.Mesh(markGeometry, markMaterial);
        mark.name = name;
        mark.rotation.x = side * -Math.PI / 2;
        mark.position.set(
          ROTOR.radiusM - SIZE.markInsetFromTip,
          side * (SIZE.bladeThickness / 2 + SIZE.markLift),
          0,
        );
        arm.add(mark);
      }
    }
    rotor.add(arm);
    housingCenters.push(new THREE.Vector3(
      housingRadius * Math.cos(angle), 0, -housingRadius * Math.sin(angle),
    ));
  }
  housingCenters.forEach((center, index) => {
    const next = housingCenters[(index + 1) % housingCenters.length];
    if (next) {
      rotor.add(createRod(center, next, SIZE.structureRodRadius, structure));
    }
  });
  root.add(rotor);

  root.add(createDisc(
    SIZE.instrumentHubRadius,
    SIZE.instrumentHubThickness,
    0,
    new THREE.MeshStandardMaterial({ map: texture, roughness: 0.55, metalness: 0.25 }),
  ));

  const antennaMaterial = new THREE.MeshStandardMaterial({
    color: COLOR.antenna, roughness: 0.55, metalness: 0.15,
  });
  const antennaBase = SIZE.instrumentHubRadius;
  root.add(createRod(
    new THREE.Vector3(antennaBase, 0, 0),
    new THREE.Vector3(antennaBase + SIZE.antennaLength, 0, 0),
    SIZE.antennaRadius,
    antennaMaterial,
  ));
  const antennaTip = new THREE.Mesh(
    new THREE.SphereGeometry(SIZE.antennaRadius * 1.5, 16, 12),
    antennaMaterial,
  );
  antennaTip.position.set(antennaBase + SIZE.antennaLength, 0, 0);
  root.add(antennaTip);

  root.add(createRod(
    new THREE.Vector3(0, AXLE_BOTTOM_M, 0),
    new THREE.Vector3(0, AXLE_TOP_M, 0),
    SIZE.axleRadius,
    new THREE.MeshStandardMaterial({ color: COLOR.axle, roughness: 0.45, metalness: 0.7 }),
  ));
  root.add(createRod(
    new THREE.Vector3(0, AXLE_BOTTOM_M, 0),
    new THREE.Vector3(0, AXLE_BOTTOM_M - SIZE.tetherLength, 0),
    SIZE.tetherRadius,
    new THREE.MeshStandardMaterial({ color: COLOR.tether, roughness: 0.9, metalness: 0 }),
  ));

  root.add(new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, FORWARD_ARROW_HEIGHT_M, 0),
    0.8,
    0x08090c,
    0.2,
    0.12,
  ));

  return { root, rotor, textures: [texture] };
}
