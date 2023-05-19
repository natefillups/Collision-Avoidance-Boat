using UnityEngine;
using UnityEngine.UI;

public class Globals : MonoBehaviour
{
    public static int Episode = 0;
    public static int Success = 0;
    public static int Fail = 0;
    private static Text txtDebug = null;

    public static void ScreenText()
    {
        if (txtDebug == null)
        {
            txtDebug = GameObject.Find("txtDebug").gameObject.GetComponent<Text>();
        }
        float SuccessPercent = (Success / (float)(Success + (Episode-Success))) * 100;
        if(txtDebug != null)
        {
            txtDebug.text = string.Format("Episode={0}, Success={1}, Fail={2} %{3}"
                , Episode
                , Success
                , (Episode - Success)
                , SuccessPercent.ToString("0"));
        }
    }
}
