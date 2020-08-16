using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class UpdateSliderValueText : MonoBehaviour
{
    public TextMeshProUGUI valueText;
    public Slider slider;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        valueText.text = slider.value.ToString("0.00");
    }

    //public void SetSliderText(float sliderValue)
    //{
    //    valueText.text = Mathf.Round(sliderValue *)
    //}
}
