using UnityEngine;

namespace CustomTools.Singletons
{
    public class SingletonMonoBehaviour<T> : MonoBehaviour where T : SingletonMonoBehaviour<T>
    {
        private static T _instance;

        public static T Instance
        {
            get
            {
                if (_instance == null)
                {
                    var go = new GameObject(nameof(T));
                    _instance = go.AddComponent<T>();
                    DontDestroyOnLoad(go);
                }

                return _instance;
            }
        }
    }
}