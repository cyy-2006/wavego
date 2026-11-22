using System;

/// <summary>
/// WAVEGO 实机腿部运动学移植(与 ESP32 固件 ServoCtrl.h 保持一致)。
///
/// 每条腿都是"侧摆舵机 + 双舵机四连杆"结构:
///   - wigglePlaneIK   : 侧摆(W)平面, 输入足端 (z 外展, y 向下), 输出侧摆角与平面内长度
///   - singleLegPlaneIK: 腿平面, 输入足端 (x 前后, 平面内长度), 输出大腿角与连杆端点
///   - simpleLinkageIK : 四连杆, 输入连杆端点, 输出两个舵机角
///
/// 单位约定: 长度 mm(与固件一致), 角度 deg。
/// 足端坐标 (x, y, z) 均相对髋关节:
///   x: 前后(+ 朝机体前方)
///   y: 向下(+ 在髋下方)
///   z: 外展(+ 背离机体)
///
/// 舵机角语义与固件 goalPWMSet 一致:
///   W = wiggleAlpha
///   F = 90 - 四连杆 alpha
///   B = 腿平面 betaAngle
/// </summary>
public static class WavegoLegKinematics
{
    #region 固件常量 (mm)

    public const double LinkageW = 19.15;    // 侧摆舵机到腿平面的距离
    public const double LinkageS = 12.2;     // 两个相邻舵机的间距
    public const double LinkageA = 40.0;     // 与舵机相连的连杆
    public const double LinkageB = 40.0;     // 限制方向的连杆
    public const double LinkageC = 39.8153;  // 大腿
    public const double LinkageD = 31.7750;  // 小腿
    public const double LinkageE = 30.8076;  // 脚

    public const double StandX = 16.0;       // 站立时足端前后偏移
    public const double StandY = 95.0;       // 站立时髋到足端距离
    public const double StandZ = 25.0;       // 站立时足端外展

    public const double WalkHeight = 95.0;
    public const double WalkHeightMax = 110.0;
    public const double WalkHeightMin = 75.0;
    public const double WalkLift = 9.0;
    public const double WalkRange = 40.0;
    public const double WalkAcc = 5.0;

    #endregion

    #region 预计算

    private const double Rad2Deg = 180.0 / Math.PI;
    private const double Deg2Rad = Math.PI / 180.0;
    private const double S_2 = LinkageS / 2.0;
    private const double L_CD = LinkageC + LinkageD;          // 大腿 + 小腿
    private const double L_CD2 = L_CD * L_CD;
    private const double L_E2 = LinkageE * LinkageE;
    private const double L_A2 = LinkageA * LinkageA;
    private const double L_B2 = LinkageB * LinkageB;
    private const double L_W2 = LinkageW * LinkageW;
    private static readonly double S_LEDC = Math.Sqrt(LinkageE * LinkageE + L_CD * L_CD);
    private static readonly double A_LCDE = Math.Atan(L_CD / LinkageE);

    #endregion

    #region 五杆闭链几何 (mm, 腿平面: x 向前, y 向下)

    // 实机腿部是平面五杆闭链: 机架上有两个主动铰点(相距 LinkageS),
    //   A/B 两根主动摆杆(各 LinkageA)由双舵机协同驱动,
    //   3→5 用 LinkC(39.8153), 4→5 用 LinkD(40.0), 两杆在铰点5汇合,
    //   铰点5再向下接足部(胫 LinkageD + 趾 LinkageE), 末端为弧形脚端。
    public const double LinkC = LinkageC;        // 铰点3 → 铰点5
    public const double LinkD = LinkageB;        // 铰点4 → 铰点5
    public const double FootShank = LinkageD;    // 足部胫段
    public const double FootToe = LinkageE;      // 足部趾段

    // 站立时两根主动杆 A/B 与竖直方向的夹角: 分别 ±45°(对称撑开)。
    public const double StandArmAngle = 45.0;

    // 足部相对连杆 D 的固定安装角(由站立标定, 保证足端落于 (16, 95))
    private const double FootShankOffsetDeg = 76.94;
    private const double FootToeOffsetDeg = 4.13;

    #endregion

    #region 基础工具

    private static double Clamp(double v, double lo, double hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static double ACos(double v)
    {
        return Math.Acos(Clamp(v, -1.0, 1.0));
    }

    private static double ASin(double v)
    {
        return Math.Asin(Clamp(v, -1.0, 1.0));
    }

    #endregion

    #region 固件三个 IK 函数

    /// <summary>
    /// 侧摆平面 IK: 输入足端 (外展 z, 向下 y), 输出侧摆角(deg)与平面内长度(mm)。
    /// 对应固件 wigglePlaneIK(Linkage_W, zPos, yPos, wiggleAlpha, wiggleLen)。
    /// </summary>
    public static double WigglePlaneIK(double zPos, double yPos, out double wiggleLen)
    {
        double aIn = zPos;
        double bIn = yPos;
        double alpha;

        if (bIn > 0)
        {
            double l2c = aIn * aIn + bIn * bIn;
            double lc = Math.Sqrt(l2c);
            double lambda = Math.Atan2(aIn, bIn) * Rad2Deg;
            double psi = ACos(LinkageW / lc) * Rad2Deg;
            wiggleLen = Math.Sqrt(Math.Max(0.0, l2c - L_W2));
            alpha = psi + lambda - 90.0;
        }
        else if (bIn == 0)
        {
            alpha = ASin(LinkageW / aIn) * Rad2Deg;
            wiggleLen = Math.Abs(aIn);
        }
        else
        {
            bIn = -bIn;
            double l2c = aIn * aIn + bIn * bIn;
            double lc = Math.Sqrt(l2c);
            double lambda = Math.Atan2(aIn, bIn) * Rad2Deg;
            double psi = ACos(LinkageW / lc) * Rad2Deg;
            wiggleLen = Math.Sqrt(Math.Max(0.0, l2c - L_W2));
            alpha = 90.0 - lambda + psi;
        }
        return alpha;
    }

    /// <summary>
    /// 腿平面 IK: 输入足端前后 x 与平面内长度, 输出腿部角(deg)与四连杆端点(mm)。
    /// 对应固件 singleLegPlaneIK(...)。
    /// </summary>
    private static double SingleLegPlaneIK(double xIn, double yIn,
                                           out double positionX, out double positionY)
    {
        double u = xIn + S_2;
        double bufferS = Math.Sqrt(u * u + yIn * yIn);
        if (bufferS < 1e-6) bufferS = 1e-6;

        double lambda = ACos((u * u + yIn * yIn + L_A2 - L_CD2 - L_E2)
                             / (2.0 * bufferS * LinkageA));
        double delta = Math.Atan2(u, yIn);
        double beta = lambda - delta;

        double theta = A_LCDE;
        double omega = ASin((yIn - Math.Cos(beta) * LinkageA) / S_LEDC);
        double nu = Math.PI - theta - omega;
        double dFX = Math.Cos(nu) * LinkageE;
        double dFY = Math.Sin(nu) * LinkageE;
        double mu = Math.PI / 2.0 - nu;
        double dEX = Math.Cos(mu) * LinkageD;
        double dEY = Math.Sin(mu) * LinkageD;

        positionX = xIn + dFX - dEX;
        positionY = yIn - dFY - dEY;

        return beta * Rad2Deg;
    }

    /// <summary>
    /// 四连杆 IK: 输入端点 (aIn, bIn), 输出舵机 alpha(deg)。
    /// 对应固件 simpleLinkageIK 中的 alpha。
    /// </summary>
    private static double SimpleLinkageAlpha(double aIn, double bIn)
    {
        double psi, alpha;

        if (Math.Abs(bIn) < 1e-9)
        {
            if (Math.Abs(aIn) < 1e-9) return 0.0;
            psi = ACos((L_A2 - L_B2 + aIn * aIn) / (2.0 * LinkageA * aIn)) * Rad2Deg;
            alpha = 90.0 - psi;
        }
        else
        {
            double l2c = aIn * aIn + bIn * bIn;
            double lc = Math.Sqrt(l2c);
            if (lc < 1e-6) lc = 1e-6;
            double lambda = Math.Atan2(bIn, aIn) * Rad2Deg;
            psi = ACos((L_A2 - L_B2 + l2c) / (2.0 * LinkageA * lc)) * Rad2Deg;
            alpha = 90.0 - lambda - psi;
        }
        return alpha;
    }

    #endregion

    #region 正/反解

    /// <summary>
    /// 足端(mm) -> 实机三舵机角(deg)。顺序 (W, F, B), 与固件 singleLegCtrl 一致。
    /// </summary>
    public static void FootToServo(double xMm, double yMm, double zMm,
                                   out double w, out double f, out double b)
    {
        double wiggleLen;
        w = WigglePlaneIK(zMm, yMm, out wiggleLen);

        double px, py;
        b = SingleLegPlaneIK(xMm, wiggleLen, out px, out py);
        f = 90.0 - SimpleLinkageAlpha(py, px - S_2);
    }

    /// <summary>
    /// 实机三舵机角(deg) -> 足端(mm)。
    /// 先用 2 变量牛顿迭代由 (B, F) 解出 (x, 平面内长度), 再由 (W, 长度) 闭式解出 (y, z)。
    /// </summary>
    public static void ServoToFoot(double w, double f, double b,
                                   out double xMm, out double yMm, out double zMm)
    {
        double alphaTarget = 90.0 - f;

        // (x, 平面内长度) 初值: 站立姿态附近
        double x = 0.0;
        double len = StandY;

        for (int iter = 0; iter < 24; iter++)
        {
            double r = SolveResidual(x, len, b, alphaTarget);
            if (r < 0.05) break;

            double h = 0.5;
            double r1 = SingleLegPlaneIK(x, len, out double t1x, out double t1y) - b;
            double r2 = SimpleLinkageAlpha(t1y, t1x - S_2) - alphaTarget;
            double j11 = (SingleLegPlaneIK(x + h, len, out double t2x, out double t2y) - b - r1) / h;
            double j21 = (SimpleLinkageAlpha(t2y, t2x - S_2) - alphaTarget - r2) / h;
            double j12 = (SingleLegPlaneIK(x, len + h, out double t3x, out double t3y) - b - r1) / h;
            double j22 = (SimpleLinkageAlpha(t3y, t3x - S_2) - alphaTarget - r2) / h;

            double det = j11 * j22 - j12 * j21;
            if (Math.Abs(det) < 1e-9) break;

            double dx = (-r1 * j22 + j12 * r2) / det;
            double dl = (-j11 * r2 + j21 * r1) / det;

            x = Clamp(x + Clamp(dx, -5.0, 5.0), -80.0, 80.0);
            len = Clamp(len + Clamp(dl, -5.0, 5.0), 20.0, 125.0);
        }

        // (W, 平面内长度) -> 平面坐标 (外展 z, 向下 y)
        double l2c = len * len + L_W2;
        double lc = Math.Sqrt(l2c);
        double psi = ACos(LinkageW / lc) * Rad2Deg;
        double lambda = (w + 90.0 - psi) * Deg2Rad;

        xMm = x;
        yMm = lc * Math.Cos(lambda);
        zMm = lc * Math.Sin(lambda);
    }

    private static double SolveResidual(double x, double len, double bTarget,
                                        double alphaTarget)
    {
        double beta = SingleLegPlaneIK(x, len, out double px, out double py);
        double alpha = SimpleLinkageAlpha(py, px - S_2);
        return Math.Abs(beta - bTarget) + Math.Abs(alpha - alphaTarget);
    }

    /// <summary>
    /// 五杆闭链正解: 由两根主动杆角度 (psiA, psiB) 求出各铰点与足端位置(mm)。
    /// psiA/psiB 以腿平面 +x(向前)为 0°, 向 +y(向下)为正, 站立时分别为 45°/135°。
    /// 铰点5取圆交点中 y 较大的一支(膝关节向下), 足部随连杆 D 刚性转动。
    /// </summary>
    public static bool FiveBarForward(double psiA, double psiB,
                                      out double x3, out double y3,
                                      out double x4, out double y4,
                                      out double x5, out double y5,
                                      out double xBend, out double yBend,
                                      out double xToe, out double yToe)
    {
        double ra = psiA * Deg2Rad;
        double rb = psiB * Deg2Rad;

        // 两个主动杆末端: 铰点3(前舵机)、铰点4(后舵机)
        x3 = S_2 + LinkageA * Math.Cos(ra);
        y3 = LinkageA * Math.Sin(ra);
        x4 = -S_2 + LinkageA * Math.Cos(rb);
        y4 = LinkageA * Math.Sin(rb);

        // 铰点5 = 以铰点3/4为圆心、LinkC/LinkD 为半径的两圆交点
        double dx = x4 - x3;
        double dy = y4 - y3;
        double d = Math.Sqrt(dx * dx + dy * dy);
        if (d < 1e-9 || d > LinkC + LinkD || d < Math.Abs(LinkC - LinkD))
        {
            x5 = y5 = xBend = yBend = xToe = yToe = 0.0;
            return false;
        }

        double a = (LinkC * LinkC - LinkD * LinkD + d * d) / (2.0 * d);
        double h = Math.Sqrt(Math.Max(0.0, LinkC * LinkC - a * a));
        double xm = x3 + a * dx / d;
        double ym = y3 + a * dy / d;
        double ux = -dy / d;
        double uy = dx / d;
        double x5a = xm + h * ux, y5a = ym + h * uy;
        double x5b = xm - h * ux, y5b = ym - h * uy;
        if (y5a >= y5b) { x5 = x5a; y5 = y5a; }
        else { x5 = x5b; y5 = y5b; }

        // 足部(胫 + 趾)作为刚性件固定在连杆 D 上
        double angD = Math.Atan2(y5 - y4, x5 - x4) * Rad2Deg;
        double sa = (angD + FootShankOffsetDeg) * Deg2Rad;
        xBend = x5 + FootShank * Math.Cos(sa);
        yBend = y5 + FootShank * Math.Sin(sa);
        double ta = (angD + FootToeOffsetDeg) * Deg2Rad;
        xToe = xBend + FootToe * Math.Cos(ta);
        yToe = yBend + FootToe * Math.Sin(ta);
        return true;
    }

    /// <summary>
    /// 五杆闭链反解: 由足端(mm)求两根主动杆角度。psiA/psiB 同时作为迭代初值。
    /// 用数值雅可比牛顿迭代, 初值取站立装配可保证始终落在同一装配支路。
    /// </summary>
    public static void FiveBarInverse(double xToe, double yToe,
                                      ref double psiA, ref double psiB)
    {
        for (int iter = 0; iter < 30; iter++)
        {
            if (!FiveBarForward(psiA, psiB, out _, out _, out _, out _,
                                out _, out _, out _, out _, out double tx, out double ty))
                return;

            double rx = tx - xToe;
            double ry = ty - yToe;
            if (Math.Abs(rx) + Math.Abs(ry) < 1e-4) return;

            const double h = 0.5;
            if (!FiveBarForward(psiA + h, psiB, out _, out _, out _, out _,
                                out _, out _, out _, out _, out double ax, out double ay))
                return;
            if (!FiveBarForward(psiA, psiB + h, out _, out _, out _, out _,
                                out _, out _, out _, out _, out double bx, out double by))
                return;

            double j11 = (ax - tx) / h, j21 = (ay - ty) / h;
            double j12 = (bx - tx) / h, j22 = (by - ty) / h;
            double det = j11 * j22 - j12 * j21;
            if (Math.Abs(det) < 1e-9) return;

            double da = (-rx * j22 + j12 * ry) / det;
            double db = (-j11 * ry + j21 * rx) / det;
            psiA += Clamp(da, -5.0, 5.0);
            psiB += Clamp(db, -5.0, 5.0);
            if (Math.Abs(da) < 1e-4 && Math.Abs(db) < 1e-4) return;
        }
    }

    /// <summary>
    /// 三维足端(mm) -> 孪生体三关节角(deg): (侧摆髋, 主动杆A, 主动杆B)。
    /// 侧摆由 wigglePlaneIK 求得, 腿平面内由五杆闭链反解求得。
    /// </summary>
    public static void FootToLinkAngles(double xMm, double yMm, double zMm,
                                        out double hipDeg, out double psiA, out double psiB)
    {
        double wiggleLen;
        hipDeg = WigglePlaneIK(zMm, yMm, out wiggleLen);

        // 站立装配初值: A/B 与竖直 ±45°
        psiA = 90.0 - StandArmAngle;   // 45°
        psiB = 90.0 + StandArmAngle;   // 135°
        FiveBarInverse(xMm, wiggleLen, ref psiA, ref psiB);
    }

    #endregion
}
