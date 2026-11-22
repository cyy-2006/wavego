using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 四足机器狗模型构建器(按 WAVEGO 实机五杆闭链腿部重建)。
///
/// 每条腿由三个舵机驱动:
///   - 侧摆髋(W): 绕机架前后轴转动, 驱动整条腿外展
///   - 腿平面内两个舵机(A/B): 相距 LinkageS, 协同驱动平面五杆闭链
///
/// 平面五杆闭链拓扑(与官方机械图一致):
///   机架点1(前舵机) --连杆A--> 铰点3 --连杆C--> 铰点5
///   机架点2(后舵机) --连杆B--> 铰点4 --连杆D--> 铰点5
///   铰点5 向下接足部(胫 LinkageD + 趾 LinkageE), 末端为弧形脚端
///   连杆 A/B 为双输入主动摆杆, 站立时与竖直方向成 ±45°。
///
/// 12 个关节值顺序: 每腿 (侧摆髋, 主动杆A, 主动杆B), 共 4 腿。
/// 模型的连杆每帧由五杆正解重新摆放, 不依赖任何外部 FBX。
/// </summary>
public class QuadrupedBuilder : MonoBehaviour
{
    #region 机体尺寸 (m)

    [Header("机体尺寸 (m)")]
    public float bodyLength = 0.22f;        // Z 方向(前后)
    public float bodyWidth = 0.10f;         // X 方向(左右)
    public float bodyThickness = 0.05f;     // Y 方向(上下)

    [Header("髋关节安装位置 (m)")]
    public float hipForwardOffset = 0.10f;  // 前后
    public float hipLateralOffset = 0.05f;  // 左右
    public float hipVerticalOffset = -0.01f;// 相对躯干中心的高度

    [Header("腿部尺寸 (m)")]
    public float hipLinkOffset = 0.01915f;  // Linkage_W: 侧摆髋到腿平面的横向连杆
    public float footRadius = 0.01f;

    #endregion

    #region 站立足端

    [Header("站立足端 (m, 相对髋关节)")]
    public float standFootX = 0.016f;       // 前后(前腿 +, 后腿 -)
    public float standFootY = 0.095f;       // 向下
    public float standFootZ = 0.025f;       // 外展

    #endregion

    #region 外观

    [Header("外观")]
    public Color bodyColor = new Color(0.25f, 0.25f, 0.28f);
    public Color legColor = new Color(0.60f, 0.60f, 0.62f);
    public Color jointColor = new Color(0.90f, 0.40f, 0.10f);
    public Color servoColor = new Color(0.15f, 0.15f, 0.18f);

    #endregion

    #region 公开常量

    public static readonly string[] LegPrefixes = { "FL", "FR", "RL", "RR" };

    // 关节角限位 (deg): 侧摆髋 ±40°, 两根主动杆 0~180°
    public static readonly Vector2[] JointLimits =
    {
        new Vector2(-40f, 40f), new Vector2(0f, 180f), new Vector2(0f, 180f),
        new Vector2(-40f, 40f), new Vector2(0f, 180f), new Vector2(0f, 180f),
        new Vector2(-40f, 40f), new Vector2(0f, 180f), new Vector2(0f, 180f),
        new Vector2(-40f, 40f), new Vector2(0f, 180f), new Vector2(0f, 180f),
    };

    #endregion

    #region 运行时状态

    public Transform Body { get; private set; }

    /// <summary>一条腿在场景中的运动学/几何引用。</summary>
    private class LegRig
    {
        public Transform hip;        // 侧摆髋关节
        public Transform plane;      // 腿平面根节点(横向偏移 Linkage_W)
        public Transform rodA, rodB, rodC, rodD;
        public Transform shank, toe;
        public Transform joint3, joint4, joint5, jointBend, jointToe;
    }

    private readonly LegRig[] _legs = new LegRig[4];
    private readonly Quaternion[] _initHipRot = new Quaternion[4];
    private readonly Vector3[] _hipAxes = new Vector3[4];
    private readonly Dictionary<string, Material> _materials = new Dictionary<string, Material>();
    private bool _built;

    /// <summary>站立时机体中心离地高度(m)</summary>
    public float StandHeight
    {
        get { return standFootY + footRadius - hipVerticalOffset; }
    }

    /// <summary>机体高度可调下限(m, 对应固件 WALK_HEIGHT_MIN)</summary>
    public float MinBodyHeight
    {
        get
        {
            return (float)WavegoLegKinematics.WalkHeightMin * 0.001f
                   + footRadius - hipVerticalOffset;
        }
    }

    /// <summary>机体高度可调上限(m, 对应固件 WALK_HEIGHT_MAX)</summary>
    public float MaxBodyHeight
    {
        get
        {
            return (float)WavegoLegKinematics.WalkHeightMax * 0.001f
                   + footRadius - hipVerticalOffset;
        }
    }

    #endregion

    #region 构建

    /// <summary>幂等构建: 重复调用不会重复生成模型。</summary>
    public void Build()
    {
        if (_built) return;
        _built = true;

        Body = NewNode("base_link", transform, new Vector3(0f, StandHeight, 0f));
        CreateBox(Body, "base_mesh",
            new Vector3(bodyWidth, bodyThickness, bodyLength),
            Vector3.zero, bodyColor);

        for (int leg = 0; leg < 4; leg++)
        {
            string p = LegPrefixes[leg];
            float forwardSign = leg < 2 ? 1f : -1f;                 // FL/FR 在前, RL/RR 在后
            float lateralSign = (leg == 0 || leg == 2) ? 1f : -1f;  // FL/RL 在左

            // ---- 侧摆髋 (W 舵机, 外展) ----
            Transform hip = NewNode(p + "_hip_link", Body,
                new Vector3(lateralSign * hipLateralOffset,
                            hipVerticalOffset,
                            forwardSign * hipForwardOffset));
            Quaternion hipBoxRot = lateralSign > 0f ? Quaternion.Euler(0f, 90f, 0f)
                                                    : Quaternion.Euler(0f, -90f, 0f);
            CreateBox(hip, "hip_servo_mesh",
                new Vector3(0.023f, 0.012f, 0.025f),
                Vector3.zero, hipBoxRot, servoColor);

            // ---- 腿平面原点: 距侧摆髋横向 Linkage_W ----
            Transform plane = NewNode(p + "_leg_plane", hip,
                new Vector3(lateralSign * hipLinkOffset, 0f, 0f));

            // 腿平面内两个舵机, 沿前后方向相距 Linkage_S
            float halfS = (float)WavegoLegKinematics.LinkageS * 0.0005f;
            CreateBox(plane, "servo_a_mesh",
                new Vector3(0.018f, 0.020f, 0.022f),
                new Vector3(0f, 0f, halfS), hipBoxRot, servoColor);
            CreateBox(plane, "servo_b_mesh",
                new Vector3(0.018f, 0.020f, 0.022f),
                new Vector3(0f, 0f, -halfS), hipBoxRot, servoColor);

            // ---- 五杆闭链各杆件与铰点(每帧由 ApplyJointAngles 摆放) ----
            var rig = new LegRig
            {
                hip = hip,
                plane = plane,
                rodA = CreateRod(plane, p + "_link_A", jointColor),
                rodB = CreateRod(plane, p + "_link_B", jointColor),
                rodC = CreateRod(plane, p + "_link_C", legColor),
                rodD = CreateRod(plane, p + "_link_D", legColor),
                shank = CreateRod(plane, p + "_foot_shank", legColor),
                toe = CreateRod(plane, p + "_foot_toe", legColor),
                joint3 = CreateJoint(plane, p + "_joint3", 0.013f),
                joint4 = CreateJoint(plane, p + "_joint4", 0.013f),
                joint5 = CreateJoint(plane, p + "_joint5", 0.014f),
                jointBend = CreateJoint(plane, p + "_ankle", 0.011f),
                jointToe = CreateJoint(plane, p + "_toe", footRadius),
            };
            _legs[leg] = rig;

            _initHipRot[leg] = hip.localRotation;
            _hipAxes[leg] = Vector3.forward * lateralSign;
        }

        ApplyJointAngles(StandPose());
    }

    /// <summary>12 关节站立姿态角(deg), 顺序与 JointLimits 一致。</summary>
    public float[] StandPose()
    {
        var pose = new float[12];
        for (int leg = 0; leg < 4; leg++)
        {
            float forwardSign = leg < 2 ? 1f : -1f;
            double hipDeg, psiA, psiB;
            WavegoLegKinematics.FootToLinkAngles(
                standFootX * 1000f * forwardSign,
                standFootY * 1000f,
                standFootZ * 1000f,
                out hipDeg, out psiA, out psiB);

            pose[leg * 3 + 0] = (float)hipDeg;
            pose[leg * 3 + 1] = (float)psiA;
            pose[leg * 3 + 2] = (float)psiB;
        }
        return pose;
    }

    /// <summary>把 12 个关节角(deg)施加到模型上。</summary>
    public void ApplyJointAngles(float[] deg)
    {
        if (!_built || deg == null) return;

        for (int leg = 0; leg < 4; leg++)
        {
            LegRig rig = _legs[leg];
            if (rig == null) continue;

            int i = leg * 3;

            // 侧摆髋
            float hipA = Mathf.Clamp(deg[i], JointLimits[i].x, JointLimits[i].y);
            rig.hip.localRotation = _initHipRot[leg] * Quaternion.AngleAxis(hipA, _hipAxes[leg]);

            // 腿平面内五杆正解
            double psiA = Mathf.Clamp(deg[i + 1], JointLimits[i + 1].x, JointLimits[i + 1].y);
            double psiB = Mathf.Clamp(deg[i + 2], JointLimits[i + 2].x, JointLimits[i + 2].y);

            if (!WavegoLegKinematics.FiveBarForward(psiA, psiB,
                    out double x3, out double y3, out double x4, out double y4,
                    out double x5, out double y5,
                    out double xBend, out double yBend,
                    out double xToe, out double yToe))
                continue;

            Vector3 p1 = PlaneToLocal(WavegoLegKinematics.LinkageS * 0.5, 0.0);
            Vector3 p2 = PlaneToLocal(-WavegoLegKinematics.LinkageS * 0.5, 0.0);
            Vector3 p3 = PlaneToLocal(x3, y3);
            Vector3 p4 = PlaneToLocal(x4, y4);
            Vector3 p5 = PlaneToLocal(x5, y5);
            Vector3 bend = PlaneToLocal(xBend, yBend);
            Vector3 toe = PlaneToLocal(xToe, yToe);

            PlaceSegment(rig.rodA, p1, p3, 0.0045f);
            PlaceSegment(rig.rodB, p2, p4, 0.0045f);
            PlaceSegment(rig.rodC, p3, p5, 0.0045f);
            PlaceSegment(rig.rodD, p4, p5, 0.0045f);
            PlaceSegment(rig.shank, p5, bend, 0.0055f);
            PlaceSegment(rig.toe, bend, toe, 0.0045f);

            PlaceJoint(rig.joint3, p3);
            PlaceJoint(rig.joint4, p4);
            PlaceJoint(rig.joint5, p5);
            PlaceJoint(rig.jointBend, bend);
            PlaceJoint(rig.jointToe, toe);
        }
    }

    /// <summary>设置机体(躯干)离地高度。</summary>
    public void SetBodyHeight(float height)
    {
        if (Body == null) return;
        Vector3 p = Body.localPosition;
        p.y = Mathf.Max(0.02f, height);
        Body.localPosition = p;
    }

    /// <summary>四个脚端球心中最低的世界高度(m), 用于镜像时把脚压回地面。</summary>
    public float LowestFootY()
    {
        float min = float.MaxValue;
        for (int leg = 0; leg < 4; leg++)
        {
            LegRig rig = _legs[leg];
            if (rig == null || rig.jointToe == null) continue;
            float y = rig.jointToe.position.y;
            if (y < min) min = y;
        }
        return min;
    }

    #endregion

    #region 几何体工具

    /// <summary>腿平面坐标(mm, x 向前 / y 向下) -> 腿平面节点局部坐标(m)。</summary>
    private static Vector3 PlaneToLocal(double xMm, double yMm)
    {
        return new Vector3(0f, (float)(-yMm * 0.001), (float)(xMm * 0.001));
    }

    /// <summary>把圆柱杆件摆放到 a→b 之间(局部坐标)。</summary>
    private static void PlaceSegment(Transform t, Vector3 a, Vector3 b, float radius)
    {
        Vector3 delta = b - a;
        float len = delta.magnitude;
        if (len < 1e-6f) return;

        t.localPosition = (a + b) * 0.5f;
        t.localRotation = Quaternion.FromToRotation(Vector3.up, delta / len);
        t.localScale = new Vector3(radius * 2f, len * 0.5f, radius * 2f);
    }

    private static void PlaceJoint(Transform t, Vector3 p)
    {
        t.localPosition = p;
    }

    private Transform NewNode(string name, Transform parent, Vector3 localPosition)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPosition;
        go.transform.localRotation = Quaternion.identity;
        go.transform.localScale = Vector3.one;
        return go.transform;
    }

    /// <summary>创建作为连杆使用的圆柱(长度/朝向由 PlaceSegment 每帧设置)。</summary>
    private Transform CreateRod(Transform parent, string name, Color color)
    {
        GameObject go = CreatePrimitive(PrimitiveType.Cylinder, parent, name,
                                        Vector3.zero, Vector3.one,
                                        Quaternion.identity, color);
        return go.transform;
    }

    /// <summary>创建一个关节球。</summary>
    private Transform CreateJoint(Transform parent, string name, float radius)
    {
        GameObject go = CreatePrimitive(PrimitiveType.Sphere, parent, name,
                                        Vector3.zero, Vector3.one * (radius * 2f),
                                        Quaternion.identity, jointColor);
        return go.transform;
    }

    private GameObject CreateBox(Transform parent, string name, Vector3 size,
                                 Vector3 localPos, Color color)
    {
        return CreateBox(parent, name, size, localPos, Quaternion.identity, color);
    }

    private GameObject CreateBox(Transform parent, string name, Vector3 size,
                                 Vector3 localPos, Quaternion localRot, Color color)
    {
        return CreatePrimitive(PrimitiveType.Cube, parent, name, localPos,
                               size, localRot, color);
    }

    private GameObject CreatePrimitive(PrimitiveType type, Transform parent, string name,
                                       Vector3 localPos, Vector3 localScale,
                                       Quaternion localRot, Color color)
    {
        GameObject go = GameObject.CreatePrimitive(type);
        go.name = name;

        // 纯运动学展示, 不需要碰撞体
        Collider collider = go.GetComponent<Collider>();
        if (collider != null) Destroy(collider);

        Transform t = go.transform;
        t.SetParent(parent, false);
        t.localPosition = localPos;
        t.localRotation = localRot;
        t.localScale = localScale;

        Renderer renderer = go.GetComponent<Renderer>();
        if (renderer != null) renderer.sharedMaterial = GetMaterial(color);
        return go;
    }

    private Material GetMaterial(Color color)
    {
        string key = ColorUtility.ToHtmlStringRGBA(color);
        if (_materials.TryGetValue(key, out Material mat) && mat != null) return mat;

        Shader shader = Shader.Find("Standard");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        mat = new Material(shader);
        mat.color = color;
        _materials[key] = mat;
        return mat;
    }

    #endregion
}
