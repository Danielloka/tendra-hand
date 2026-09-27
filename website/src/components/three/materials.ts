import { BackSide, Color, LineBasicMaterial, type Material, MeshBasicMaterial, MeshStandardMaterial } from "three";

/**
 * Colours for the scene, read from the design-system tokens at runtime so the
 * hand follows the light/dark theme (no hex values in code).
 */
export type Palette = {
  dark: boolean;
  bg: string;
  body: string;
  pin: string;
  line: string;
  accent: string;
};

export function readPalette(): Palette {
  const root = document.documentElement;
  const css = getComputedStyle(root);
  const token = (name: string) => css.getPropertyValue(name).trim();
  const dark = root.dataset.theme === "dark";
  return {
    dark,
    bg: token("--bg"),
    // White PLA in both themes: the light surface tint on white pages, the near-white text colour on black.
    body: dark ? token("--text") : token("--bg-alt"),
    pin: dark ? token("--border-strong") : token("--text-muted"),
    line: token("--text-muted"),
    accent: token("--accent"),
  };
}

function setColor(color: Color, value: string) {
  if (value) color.setStyle(value);
}

type Solid = { material: Material; base: number };

/** Outline thickness as a fraction of the camera distance (≈ constant on screen). */
const OUTLINE_WIDTH = 0.0016;

/**
 * Shared materials for one canvas. Every body material gets a view-dependent
 * (fresnel) rim term in the accent colour, which reads as a calm blue edge
 * glow even on a white page. Wireframe mode fades the solids out; they keep
 * writing depth, so the outlines + edge lines become a clean hidden-line drawing.
 */
export class HandMaterials {
  readonly rimUniforms = {
    uRimColor: { value: new Color() },
    uRimStrength: { value: 0 },
  };
  readonly accent = new Color();

  /** Parts that exist on the real prototype (palm, thumb, index). */
  readonly real = new MeshStandardMaterial({ roughness: 0.62, metalness: 0 });
  /** Planned fingers: a touch see-through. */
  readonly planned = new MeshStandardMaterial({ roughness: 0.7, metalness: 0, transparent: true, opacity: 0.7 });
  /** Joint pins/axles. */
  readonly pin = new MeshStandardMaterial({ roughness: 0.38, metalness: 0.2 });
  /** Wireframe view: sharp feature edges (pins, rims). */
  readonly line = new LineBasicMaterial({ transparent: true, opacity: 0, depthWrite: false, visible: false });
  /** Wireframe view: silhouettes, drawn as an inflated back-face shell ("inverted hull"). */
  readonly outline = new MeshBasicMaterial({ side: BackSide, transparent: true, opacity: 0, depthWrite: false, visible: false });
  readonly tendonCore = new MeshBasicMaterial({ transparent: true, depthWrite: false, toneMapped: false });
  readonly tendonHalo = new MeshBasicMaterial({ transparent: true, depthWrite: false, toneMapped: false });

  private readonly solids: Solid[] = [];
  private wireframe = -1;

  constructor() {
    for (const m of [this.real, this.planned, this.pin]) this.addSolid(m);
    this.outline.onBeforeCompile = (shader) => {
      shader.vertexShader = shader.vertexShader.replace(
        "#include <project_vertex>",
        `#include <project_vertex>
        mvPosition.xyz += normalize(normalMatrix * normal) * (${OUTLINE_WIDTH} * -mvPosition.z);
        gl_Position = projectionMatrix * mvPosition;`,
      );
    };
  }

  /** Registers a body material: rim glow + wireframe fade. Returns it for chaining. */
  addSolid<T extends Material>(material: T): T {
    material.polygonOffset = true; // push faces back so edge lines don't z-fight
    material.polygonOffsetFactor = 1;
    material.polygonOffsetUnits = 1;
    if (material instanceof MeshStandardMaterial) this.patchRim(material);
    this.solids.push({ material, base: material.transparent ? material.opacity : 1 });
    return material;
  }

  private patchRim(material: MeshStandardMaterial) {
    material.onBeforeCompile = (shader) => {
      Object.assign(shader.uniforms, this.rimUniforms);
      shader.fragmentShader = shader.fragmentShader
        .replace("#include <common>", "#include <common>\nuniform vec3 uRimColor;\nuniform float uRimStrength;")
        .replace(
          "#include <emissivemap_fragment>",
          `#include <emissivemap_fragment>
          float rimF = 1.0 - saturate(dot(normal, normalize(vViewPosition)));
          totalEmissiveRadiance += uRimColor * uRimStrength * pow(rimF, 2.6);`,
        );
    };
    material.customProgramCacheKey = () => "tendra-rim";
    material.needsUpdate = true;
  }

  setPalette(p: Palette) {
    setColor(this.real.color, p.body);
    // Planned fingers: see-through on white; on black (where see-through white turns muddy grey) opaque and a little dimmer.
    setColor(this.planned.color, p.body);
    if (p.dark && p.bg) this.planned.color.lerp(new Color().setStyle(p.bg), 0.22);
    setColor(this.pin.color, p.pin);
    setColor(this.line.color, p.line);
    setColor(this.outline.color, p.line);
    setColor(this.accent, p.accent);
    this.rimUniforms.uRimColor.value.copy(this.accent);
    this.tendonCore.color.copy(this.accent);
    this.tendonHalo.color.copy(this.accent);
    const planned = this.solids.find((s) => s.material === this.planned);
    if (planned) planned.base = p.dark ? 1 : 0.7;
    this.applyWireframe(Math.max(this.wireframe, 0));
  }

  setRim(rim: number) {
    this.rimUniforms.uRimStrength.value = rim * 0.85;
  }

  /** 0 = solid, 1 = line drawing. */
  setWireframe(w: number) {
    if (w !== this.wireframe) this.applyWireframe(w);
  }

  private applyWireframe(w: number) {
    this.wireframe = w;
    for (const { material, base } of this.solids) {
      const opacity = base * (1 - w);
      const transparent = opacity < 0.999;
      if (material.transparent !== transparent) {
        material.transparent = transparent;
        material.needsUpdate = true;
      }
      material.opacity = opacity;
      material.colorWrite = opacity > 0.001; // still writes depth: hides back lines
    }
    for (const m of [this.line, this.outline]) {
      m.opacity = w;
      m.visible = w > 0.001;
    }
  }

  dispose() {
    for (const { material } of this.solids) material.dispose();
    for (const m of [this.line, this.outline, this.tendonCore, this.tendonHalo]) m.dispose();
  }
}
