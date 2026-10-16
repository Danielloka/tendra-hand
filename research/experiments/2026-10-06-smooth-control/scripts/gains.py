from tendra.scene import GraspScene, SceneConfig

s = GraspScene(SceneConfig())
m = s.model
print("timestep", m.opt.timestep, "integrator", m.opt.integrator)
for side in ("right",):
    arm = s.sides[side]
    for i in list(arm.arm_act) + list(arm.act[:4]):
        j = m.actuator_trnid[i,0]
        print(f"{m.actuator(i).name:28s} gain {m.actuator_gainprm[i,0]:8.3f} bias {m.actuator_biasprm[i,:3]} frc {m.actuator_forcerange[i]} trntype {m.actuator_trntype[i]} dyn {m.actuator_dyntype[i]}")
    for n in arm.arm_qadr: pass
    dofs = arm.arm_dofs
    print("arm dof damping", m.dof_damping[dofs], "armature", m.dof_armature[dofs], "frictionloss", m.dof_frictionloss[dofs])
    print("finger dof damping", m.dof_damping[[m.jnt_dofadr[m.joint(f'{side}_{n}').id] for n in ['index_pip','index_mcp_flex','thumb_cmc_rot']]])
    print("gravcomp arm bodies", set(m.body_gravcomp[m.body_rootid == m.body_rootid[m.site_bodyid[arm.site]]]))
