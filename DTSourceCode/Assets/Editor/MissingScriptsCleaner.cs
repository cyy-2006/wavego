using UnityEngine;
using UnityEditor;

/// <summary>
/// 编辑器工具：清理场景中丢失的脚本
/// </summary>
public class MissingScriptsCleaner : EditorWindow
{
    [MenuItem("Tools/清理丢失的脚本")]
    public static void ShowWindow()
    {
        GetWindow<MissingScriptsCleaner>("清理丢失脚本");
    }

    void OnGUI()
    {
        GUILayout.Label("清理场景中丢失的脚本", EditorStyles.boldLabel);
        GUILayout.Space(10);
        
        GUILayout.Label("此工具将删除场景中所有丢失脚本的组件引用。");
        GUILayout.Label("这通常是因为game4automation框架被移除导致的。");
        GUILayout.Space(10);
        
        if (GUILayout.Button("查找丢失的脚本", GUILayout.Height(30)))
        {
            FindMissingScripts();
        }
        
        GUILayout.Space(10);
        
        GUI.backgroundColor = Color.red;
        if (GUILayout.Button("删除所有丢失的脚本", GUILayout.Height(40)))
        {
            if (EditorUtility.DisplayDialog("确认删除", 
                "确定要删除场景中所有丢失的脚本组件吗？\n\n此操作不可撤销，请先保存场景备份！", 
                "删除", "取消"))
            {
                RemoveMissingScripts();
            }
        }
        GUI.backgroundColor = Color.white;
    }

    static void FindMissingScripts()
    {
        int count = 0;
        GameObject[] allObjects = GameObject.FindObjectsOfType<GameObject>();
        
        foreach (GameObject go in allObjects)
        {
            Component[] components = go.GetComponents<Component>();
            foreach (Component c in components)
            {
                if (c == null)
                {
                    count++;
                    Debug.Log($"<color=yellow>发现丢失脚本: {GetFullPath(go)}</color>");
                }
            }
        }
        
        if (count == 0)
        {
            Debug.Log("<color=green>未发现丢失的脚本！</color>");
        }
        else
        {
            Debug.Log($"<color=orange>共发现 {count} 个丢失的脚本组件</color>");
        }
    }

    static void RemoveMissingScripts()
    {
        int totalRemoved = 0;
        GameObject[] allObjects = GameObject.FindObjectsOfType<GameObject>();
        
        foreach (GameObject go in allObjects)
        {
            int removed = GameObjectUtility.RemoveMonoBehavioursWithMissingScript(go);
            if (removed > 0)
            {
                totalRemoved += removed;
                Debug.Log($"<color=cyan>已从 {go.name} 移除 {removed} 个丢失脚本</color>");
                EditorUtility.SetDirty(go);
            }
        }
        
        if (totalRemoved > 0)
        {
            Debug.Log($"<color=green>完成！共移除 {totalRemoved} 个丢失的脚本组件</color>");
            Debug.Log("<color=yellow>请保存场景以保留更改 (Ctrl+S)</color>");
        }
        else
        {
            Debug.Log("<color=green>未发现需要移除的丢失脚本</color>");
        }
    }
    
    static string GetFullPath(GameObject go)
    {
        string path = go.name;
        Transform parent = go.transform.parent;
        while (parent != null)
        {
            path = parent.name + "/" + path;
            parent = parent.parent;
        }
        return path;
    }
}
