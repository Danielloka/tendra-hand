import { BufferAttribute, BufferGeometry, CatmullRomCurve3, Mesh, type Object3D, Vector3 } from "three";
import type { HandMaterials } from "./materials";
import type { Anchor } from "./rig";

const SAMPLES = 72; // points along each tendon
const RADIAL = 6; // sides of the tube
const CORE_RADIUS = 0.00075;
const HALO_RADIUS = 0.0024;

/**
 * Glowing tendon lines along the back of every finger. The anchors move with
 * the finger segments, so the tubes are re-shaped every frame; the vertex
 * buffers are reused (no allocation per frame). Triangles are ordered by
 * distance along the tendon for all fingers at once, so `drawRange` "draws
 * them on" from wrist to fingertip.
 */
export class Tendons {
  readonly group: Mesh[];
  private readonly core: Mesh;
  private readonly halo: Mesh;
  private readonly curves: CatmullRomCurve3[];
  private readonly centres: Vector3[][];
  private readonly tangents: Vector3[];
  private readonly normals: Vector3[];
  private readonly trianglesPerStep: number;

  constructor(
    private readonly paths: Anchor[][],
    private readonly mats: HandMaterials,
  ) {
    this.curves = paths.map((p) => new CatmullRomCurve3(p.map(() => new Vector3()), false, "centripetal"));
    this.centres = paths.map(() => Array.from({ length: SAMPLES }, () => new Vector3()));
    this.tangents = Array.from({ length: SAMPLES }, () => new Vector3());
    this.normals = Array.from({ length: SAMPLES }, () => new Vector3());
    this.trianglesPerStep = paths.length * RADIAL * 2;
    this.core = new Mesh(this.createGeometry(), mats.tendonCore);
    this.halo = new Mesh(this.createGeometry(), mats.tendonHalo);
    for (const m of [this.core, this.halo]) {
      m.frustumCulled = false; // bounds change every frame
      m.renderOrder = 2;
      m.visible = false;
      m.raycast = () => {};
    }
    this.group = [this.halo, this.core];
  }

  private createGeometry() {
    const g = new BufferGeometry();
    const vertsPerPath = SAMPLES * RADIAL;
    g.setAttribute("position", new BufferAttribute(new Float32Array(this.paths.length * vertsPerPath * 3), 3));
    const index: number[] = [];
    for (let s = 0; s < SAMPLES - 1; s++) {
      for (let p = 0; p < this.paths.length; p++) {
        const a0 = p * vertsPerPath + s * RADIAL;
        const b0 = a0 + RADIAL;
        for (let k = 0; k < RADIAL; k++) {
          const k1 = (k + 1) % RADIAL;
          index.push(a0 + k, b0 + k, a0 + k1, a0 + k1, b0 + k, b0 + k1);
        }
      }
    }
    g.setIndex(index);
    return g;
  }

  /**
   * @param space the object whose local space the tubes live in (their parent)
   * @param amount 0..1 from handState.tendons
   */
  update(space: Object3D, amount: number) {
    const visible = amount > 0.001;
    this.core.visible = this.halo.visible = visible;
    if (!visible) return;

    const progress = Math.min(1, amount * 1.35);
    const opacity = Math.min(1, amount * 3);
    this.mats.tendonCore.opacity = opacity;
    this.mats.tendonHalo.opacity = opacity * 0.22;
    const steps = Math.floor(progress * (SAMPLES - 1));
    for (const m of this.group) m.geometry.setDrawRange(0, steps * this.trianglesPerStep * 3);

    // Anchor points → curve → tube rings (for both layers).
    space.updateWorldMatrix(true, true);
    this.paths.forEach((path, p) => {
      const curve = this.curves[p];
      path.forEach((a, i) => space.worldToLocal(a.node.localToWorld(curve.points[i].copy(a.offset))));
      const centres = this.centres[p];
      for (let s = 0; s < SAMPLES; s++) curve.getPoint(s / (SAMPLES - 1), centres[s]);
      this.frames(centres);
      this.writeRings(this.core.geometry, p, centres, CORE_RADIUS);
      this.writeRings(this.halo.geometry, p, centres, HALO_RADIUS);
    });
    for (const m of this.group) (m.geometry.getAttribute("position") as BufferAttribute).needsUpdate = true;
  }

  /** Parallel-transport frames: tangents + a normal that doesn't twist. */
  private frames(c: Vector3[]) {
    const { tangents: t, normals: n } = this;
    for (let s = 0; s < SAMPLES; s++) {
      const a = c[Math.max(0, s - 1)];
      const b = c[Math.min(SAMPLES - 1, s + 1)];
      t[s].subVectors(b, a).normalize();
    }
    // First normal: any vector perpendicular to the first tangent.
    const ref = Math.abs(t[0].x) < 0.9 ? tmpA.set(1, 0, 0) : tmpA.set(0, 1, 0);
    n[0].crossVectors(t[0], ref).normalize();
    for (let s = 1; s < SAMPLES; s++) {
      n[s].copy(n[s - 1]).addScaledVector(t[s], -t[s].dot(n[s - 1])).normalize();
    }
  }

  private writeRings(g: BufferGeometry, p: number, c: Vector3[], radius: number) {
    const pos = (g.getAttribute("position") as BufferAttribute).array as Float32Array;
    let o = p * SAMPLES * RADIAL * 3;
    for (let s = 0; s < SAMPLES; s++) {
      const r = radius * (1 - 0.35 * (s / (SAMPLES - 1))); // thinner toward the fingertip
      const bin = tmpB.crossVectors(this.tangents[s], this.normals[s]);
      for (let k = 0; k < RADIAL; k++) {
        const ang = (k / RADIAL) * Math.PI * 2;
        const cs = Math.cos(ang) * r;
        const sn = Math.sin(ang) * r;
        pos[o++] = c[s].x + this.normals[s].x * cs + bin.x * sn;
        pos[o++] = c[s].y + this.normals[s].y * cs + bin.y * sn;
        pos[o++] = c[s].z + this.normals[s].z * cs + bin.z * sn;
      }
    }
  }

  dispose() {
    this.core.geometry.dispose();
    this.halo.geometry.dispose();
  }
}

const tmpA = new Vector3();
const tmpB = new Vector3();
