using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateAround : MonoBehaviour
{
    public Transform target; // 要围绕的目标
    public float rotationSpeed = 5f; // 鼠标拖拽旋转速度
    public float scrollSpeed = 2f; // 滚轮缩放速度
    public float smoothTime = 0.2f; // 平滑过渡时间

    [Header("Distance Settings")]
    public float minDistance = 1f; // 最小距离
    public float maxDistance = 10f; // 最大距离
    public float initialDistance = 5f; // 初始距离

    [Header("Vertical Angle Limits")]
    public float minVerticalAngle = -20f; // 最低俯角（防止摄像机翻转）
    public float maxVerticalAngle = 80f; // 最大仰角

    private float currentDistance; // 当前距离
    private float targetDistance; // 目标距离（用于平滑过渡）
    private float distanceVelocity; // 用于 SmoothDamp

    private Vector2 currentRotation; // 当前旋转角度（x=垂直，y=水平）
    private Vector2 rotationVelocity; // 用于平滑旋转

    void Start()
    {
        currentDistance = targetDistance = initialDistance;

        // 方法 1：直接使用场景摄像机的初始角度
        Vector3 euler = transform.rotation.eulerAngles;
        currentRotation = new Vector2(euler.x, euler.y);

        // 方法 3：额外限制初始角度，防止奇怪视角
        currentRotation.x = Mathf.Clamp(
            currentRotation.x,
            minVerticalAngle,
            maxVerticalAngle
        );
    }

    void LateUpdate()
    {
        if (target == null)
        {
            Debug.LogWarning("OrbitCamera: No target assigned!");
            return;
        }

        HandleMouseRotation();
        HandleMouseScroll();
        UpdateCameraPosition();
    }

    // 处理鼠标拖拽旋转
    private void HandleMouseRotation()
    {
        if (Input.GetMouseButton(0)) // 按住鼠标左键旋转
        {
            float mouseX = Input.GetAxis("Mouse X") * rotationSpeed;
            float mouseY = Input.GetAxis("Mouse Y") * rotationSpeed;

            currentRotation.y += mouseX; // 水平旋转（左右）
            currentRotation.x -= mouseY; // 垂直旋转（上下）

            // 限制垂直角度，防止摄像机翻转
            currentRotation.x = Mathf.Clamp(currentRotation.x, minVerticalAngle, maxVerticalAngle);
        }
    }

    // 处理鼠标滚轮缩放
    private void HandleMouseScroll()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0)
        {
            targetDistance -= scroll * scrollSpeed;
            targetDistance = Mathf.Clamp(targetDistance, minDistance, maxDistance);
        }

        // 平滑过渡距离
        currentDistance = Mathf.SmoothDamp(
            currentDistance,
            targetDistance,
            ref distanceVelocity,
            smoothTime
        );
    }

    // 更新摄像机位置和朝向
    private void UpdateCameraPosition()
    {
        // 计算旋转（Y轴水平旋转，X轴垂直旋转）
        Quaternion rotation = Quaternion.Euler(currentRotation.x, currentRotation.y, 0);

        // 计算摄像机位置（保持固定距离）
        Vector3 orbitPosition = target.position + rotation * Vector3.back * currentDistance;

        // 平滑移动摄像机
        transform.position = orbitPosition;
        transform.LookAt(target.position);
    }

    // 重置摄像机（可选）
    public void ResetCamera()
    {
        currentRotation = Vector2.zero;
        targetDistance = initialDistance;
    }
}
