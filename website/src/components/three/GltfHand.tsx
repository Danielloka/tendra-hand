"use client";

import { useGLTF } from "@react-three/drei";
import { useEffect, useMemo } from "react";
import { rigFromGltf } from "./gltfRig";
import { type ModelProps, RigView } from "./HandModel";

/** The real hand from HAND_MODEL_URL (see public/models/README.md). Loaded lazily by HandModel. */
export default function GltfHand({ url, ...props }: ModelProps & { url: string }) {
  const { scene } = useGLTF(url);
  const rig = useMemo(() => rigFromGltf(scene, props.mats), [scene, props.mats]);
  useEffect(() => rig.dispose, [rig]);
  return <RigView rig={rig} {...props} />;
}
