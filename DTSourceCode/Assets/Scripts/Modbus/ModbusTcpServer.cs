using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

/// <summary>
/// Modbus TCP 服务器 - 支持功能码03/04/06/16
/// </summary>
public class ModbusTcpServer : MonoBehaviour
{
    #region 配置
    
    [Header("服务器配置")]
    // Linux 下普通用户无法绑定 1024 以下端口(ip_unprivileged_port_start 默认为 1024),
    // 因此默认使用非特权端口 1502; 若需要标准的 502 端口, 见 StartServer 中的提示。
    public int Port = 1502;
    public int MaxClients = 5;
    public int HoldingRegisterCount = 100;
    public int InputRegisterCount = 100;
    
    #endregion
    
    #region 状态属性
    
    public bool IsRunning { get; private set; }
    public int ClientCount => _clients.Count;
    public static ModbusTcpServer Instance { get; private set; }
    
    #endregion
    
    #region 私有变量
    
    private ushort[] _holdingRegs;
    private ushort[] _inputRegs;
    private readonly object _lock = new object();
    
    private TcpListener _listener;
    private Thread _acceptThread;
    private List<TcpClient> _clients = new List<TcpClient>();
    private volatile bool _stop = false;
    
    #endregion
    
    #region 生命周期
    
    void Awake()
    {
        if (Instance == null) { Instance = this; DontDestroyOnLoad(gameObject); }
        else { Destroy(gameObject); return; }
        
        _holdingRegs = new ushort[HoldingRegisterCount];
        _inputRegs = new ushort[InputRegisterCount];
    }
    
    void Start() => StartServer();
    void OnDestroy() { StopServer(); if (Instance == this) Instance = null; }
    void OnApplicationQuit() => StopServer();
    
    #endregion
    
    #region 服务器控制
    
    public void StartServer()
    {
        if (IsRunning) return;
        _stop = false;

        Exception error = TryStartListener(Port);
        if (error == null) { Launch(Port); return; }

        // Linux 普通用户无法绑定 1024 以下端口, 此时自动回退到非特权端口
        if (Port < 1024)
        {
            int fallback = Port + 1000;
            error = TryStartListener(fallback);
            if (error == null)
            {
                Debug.LogWarning($"[ModbusTcpServer] 端口 {Port} 无权限(Access denied), 已自动改用 {fallback}");
                Launch(fallback);
                return;
            }
        }

        Debug.LogError($"[ModbusTcpServer] 启动失败: {error.Message}\n" +
                       $"若为 Access denied, 说明当前用户无权绑定 {Port} 端口; " +
                       $"可执行: sudo sysctl -w net.ipv4.ip_unprivileged_port_start={Port}");
    }

    private Exception TryStartListener(int port)
    {
        try
        {
            _listener = new TcpListener(IPAddress.Any, port);
            _listener.Start();
            return null;
        }
        catch (Exception ex) { return ex; }
    }

    private void Launch(int port)
    {
        IsRunning = true;
        _acceptThread = new Thread(AcceptLoop) { IsBackground = true };
        _acceptThread.Start();
        Debug.Log($"<color=green>[ModbusTcpServer] 服务器已启动: {port}</color>");
    }
    
    public void StopServer()
    {
        _stop = true; IsRunning = false;
        try { _listener?.Stop(); } catch { }
        lock (_clients) { foreach (var c in _clients) try { c?.Close(); } catch { } _clients.Clear(); }
        _acceptThread?.Join(1000);
    }
    
    #endregion
    
    #region 网络处理
    
    void AcceptLoop()
    {
        while (!_stop)
        {
            try
            {
                if (_listener.Pending())
                {
                    var client = _listener.AcceptTcpClient();
                    lock (_clients)
                    {
                        if (_clients.Count < MaxClients)
                        {
                            _clients.Add(client);
                            new Thread(() => HandleClient(client)) { IsBackground = true }.Start();
                            Debug.Log($"<color=cyan>[Modbus] 客户端连接: {client.Client.RemoteEndPoint}</color>");
                        }
                        else client.Close();
                    }
                }
                else Thread.Sleep(100);
            }
            catch { if (!_stop) break; }
        }
    }
    
    void HandleClient(TcpClient client)
    {
        try
        {
            var stream = client.GetStream();
            var buffer = new byte[256];
            while (!_stop && client.Connected)
            {
                if (stream.DataAvailable)
                {
                    int len = stream.Read(buffer, 0, buffer.Length);
                    if (len > 0)
                    {
                        var resp = ProcessRequest(buffer, len);
                        if (resp != null) stream.Write(resp, 0, resp.Length);
                    }
                }
                else Thread.Sleep(10);
            }
        }
        catch { }
        finally { lock (_clients) _clients.Remove(client); try { client?.Close(); } catch { } }
    }
    
    #endregion
    
    #region Modbus协议处理
    
    byte[] ProcessRequest(byte[] req, int len)
    {
        if (len < 12) return null;
        ushort txId = (ushort)((req[0] << 8) | req[1]);
        if ((req[2] << 8 | req[3]) != 0) return null;
        byte unit = req[6], func = req[7];
        
        return func switch
        {
            0x03 => ReadRegisters(req, txId, unit, _holdingRegs, HoldingRegisterCount, 0x03),
            0x04 => ReadRegisters(req, txId, unit, _inputRegs, InputRegisterCount, 0x04),
            0x06 => WriteSingle(req, txId, unit),
            0x10 => WriteMultiple(req, txId, unit),
            _ => new byte[] { (byte)(txId >> 8), (byte)txId, 0, 0, 0, 3, unit, (byte)(func | 0x80), 0x01 }
        };
    }
    
    byte[] ReadRegisters(byte[] req, ushort txId, byte unit, ushort[] regs, int max, byte func)
    {
        ushort addr = (ushort)((req[8] << 8) | req[9]);
        ushort qty = (ushort)((req[10] << 8) | req[11]);
        if (addr + qty > max) return new byte[] { (byte)(txId >> 8), (byte)txId, 0, 0, 0, 3, unit, (byte)(func | 0x80), 0x02 };
        
        byte bc = (byte)(qty * 2);
        var resp = new byte[9 + bc];
        resp[0] = (byte)(txId >> 8); resp[1] = (byte)txId;
        resp[4] = (byte)((3 + bc) >> 8); resp[5] = (byte)(3 + bc);
        resp[6] = unit; resp[7] = func; resp[8] = bc;
        lock (_lock) for (int i = 0; i < qty; i++) { resp[9 + i * 2] = (byte)(regs[addr + i] >> 8); resp[10 + i * 2] = (byte)regs[addr + i]; }
        return resp;
    }
    
    byte[] WriteSingle(byte[] req, ushort txId, byte unit)
    {
        ushort addr = (ushort)((req[8] << 8) | req[9]);
        ushort val = (ushort)((req[10] << 8) | req[11]);
        if (addr >= HoldingRegisterCount) return new byte[] { (byte)(txId >> 8), (byte)txId, 0, 0, 0, 3, unit, 0x86, 0x02 };
        lock (_lock) _holdingRegs[addr] = val;
        var resp = new byte[12]; Array.Copy(req, resp, 12); resp[0] = (byte)(txId >> 8); resp[1] = (byte)txId;
        return resp;
    }
    
    byte[] WriteMultiple(byte[] req, ushort txId, byte unit)
    {
        ushort addr = (ushort)((req[8] << 8) | req[9]);
        ushort qty = (ushort)((req[10] << 8) | req[11]);
        if (addr + qty > HoldingRegisterCount) return new byte[] { (byte)(txId >> 8), (byte)txId, 0, 0, 0, 3, unit, 0x90, 0x02 };
        lock (_lock) for (int i = 0; i < qty; i++) _holdingRegs[addr + i] = (ushort)((req[13 + i * 2] << 8) | req[14 + i * 2]);
        return new byte[] { (byte)(txId >> 8), (byte)txId, 0, 0, 0, 6, unit, 0x10, (byte)(addr >> 8), (byte)addr, (byte)(qty >> 8), (byte)qty };
    }
    
    #endregion
    
    #region 公共接口 - Holding Registers
    
    public ushort ReadRegister(int addr)
    {
        if (addr < 0 || addr >= HoldingRegisterCount) return 0;
        lock (_lock) return _holdingRegs[addr];
    }
    
    public void WriteRegister(int addr, ushort val)
    {
        if (addr < 0 || addr >= HoldingRegisterCount) return;
        lock (_lock) _holdingRegs[addr] = val;
    }
    
    public float ReadFloat(int addr)
    {
        if (addr < 0 || addr + 1 >= HoldingRegisterCount) return 0;
        lock (_lock) return DecodeFloat(_holdingRegs[addr], _holdingRegs[addr + 1]);
    }
    
    public void WriteFloat(int addr, float val)
    {
        if (addr < 0 || addr + 1 >= HoldingRegisterCount) return;
        var r = EncodeFloat(val);
        lock (_lock) { _holdingRegs[addr] = r[0]; _holdingRegs[addr + 1] = r[1]; }
    }
    
    #endregion
    
    #region 公共接口 - Input Registers
    
    public ushort ReadInputRegister(int addr)
    {
        if (addr < 0 || addr >= InputRegisterCount) return 0;
        lock (_lock) return _inputRegs[addr];
    }
    
    public void WriteInputRegister(int addr, ushort val)
    {
        if (addr < 0 || addr >= InputRegisterCount) return;
        lock (_lock) _inputRegs[addr] = val;
    }
    
    public float ReadInputFloat(int addr)
    {
        if (addr < 0 || addr + 1 >= InputRegisterCount) return 0;
        lock (_lock) return DecodeFloat(_inputRegs[addr], _inputRegs[addr + 1]);
    }
    
    public void WriteInputFloat(int addr, float val)
    {
        if (addr < 0 || addr + 1 >= InputRegisterCount) return;
        var r = EncodeFloat(val);
        lock (_lock) { _inputRegs[addr] = r[0]; _inputRegs[addr + 1] = r[1]; }
    }
    
    #endregion
    
    #region IEEE754编解码
    
    public static float DecodeFloat(ushort hi, ushort lo)
    {
        var b = new byte[] { (byte)lo, (byte)(lo >> 8), (byte)hi, (byte)(hi >> 8) };
        return BitConverter.ToSingle(b, 0);
    }
    
    public static ushort[] EncodeFloat(float val)
    {
        var b = BitConverter.GetBytes(val);
        return new ushort[] { (ushort)((b[3] << 8) | b[2]), (ushort)((b[1] << 8) | b[0]) };
    }
    
    #endregion
}
