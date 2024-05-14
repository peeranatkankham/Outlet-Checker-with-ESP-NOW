// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_Panel1;
lv_obj_t * ui_Panel4;
lv_obj_t * ui_Label1;
lv_obj_t * ui_status1;
void ui_event_Button1(lv_event_t * e);
lv_obj_t * ui_Button1;
lv_obj_t * ui_Image1;
lv_obj_t * ui_Panel2;
lv_obj_t * ui_Panel5;
lv_obj_t * ui_Label2;
lv_obj_t * ui_status2;
void ui_event_Button2(lv_event_t * e);
lv_obj_t * ui_Button2;
lv_obj_t * ui_Image2;
lv_obj_t * ui_Panel3;
lv_obj_t * ui_Panel6;
lv_obj_t * ui_Label3;
lv_obj_t * ui_status3;
void ui_event_Button3(lv_event_t * e);
lv_obj_t * ui_Button3;
lv_obj_t * ui_Image3;


// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
lv_obj_t * ui_Screen2;
lv_obj_t * ui_Container1;
lv_obj_t * ui_Label4;
void ui_event_ranges(lv_event_t * e);
lv_obj_t * ui_ranges;
void ui_event_washing(lv_event_t * e);
lv_obj_t * ui_washing;
void ui_event_oven(lv_event_t * e);
lv_obj_t * ui_oven;
void ui_event_bed(lv_event_t * e);
lv_obj_t * ui_bed;
void ui_event_tv(lv_event_t * e);
lv_obj_t * ui_tv;
void ui_event_sink(lv_event_t * e);
lv_obj_t * ui_sink;
void ui_event_couch(lv_event_t * e);
lv_obj_t * ui_couch;
void ui_event_pc(lv_event_t * e);
lv_obj_t * ui_pc;
void ui_event_map(lv_event_t * e);
lv_obj_t * ui_map;
void ui_event_h1(lv_event_t * e);
lv_obj_t * ui_h1;
void ui_event_h2(lv_event_t * e);
lv_obj_t * ui_h2;
void ui_event_h3(lv_event_t * e);
lv_obj_t * ui_h3;


// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
lv_obj_t * ui_Screen3;
lv_obj_t * ui_Container2;
lv_obj_t * ui_Label5;
void ui_event_ranges1(lv_event_t * e);
lv_obj_t * ui_ranges1;
void ui_event_washing1(lv_event_t * e);
lv_obj_t * ui_washing1;
void ui_event_oven1(lv_event_t * e);
lv_obj_t * ui_oven1;
void ui_event_bed1(lv_event_t * e);
lv_obj_t * ui_bed1;
void ui_event_tv1(lv_event_t * e);
lv_obj_t * ui_tv1;
void ui_event_sink1(lv_event_t * e);
lv_obj_t * ui_sink1;
void ui_event_couch1(lv_event_t * e);
lv_obj_t * ui_couch1;
void ui_event_pc1(lv_event_t * e);
lv_obj_t * ui_pc1;
void ui_event_map1(lv_event_t * e);
lv_obj_t * ui_map1;
void ui_event_h4(lv_event_t * e);
lv_obj_t * ui_h4;
void ui_event_h5(lv_event_t * e);
lv_obj_t * ui_h5;
void ui_event_h6(lv_event_t * e);
lv_obj_t * ui_h6;


// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
lv_obj_t * ui_Screen4;
lv_obj_t * ui_Container3;
lv_obj_t * ui_Label6;
void ui_event_ranges2(lv_event_t * e);
lv_obj_t * ui_ranges2;
void ui_event_washing2(lv_event_t * e);
lv_obj_t * ui_washing2;
void ui_event_oven2(lv_event_t * e);
lv_obj_t * ui_oven2;
void ui_event_bed2(lv_event_t * e);
lv_obj_t * ui_bed2;
void ui_event_tv2(lv_event_t * e);
lv_obj_t * ui_tv2;
void ui_event_sink2(lv_event_t * e);
lv_obj_t * ui_sink2;
void ui_event_couch2(lv_event_t * e);
lv_obj_t * ui_couch2;
void ui_event_pc2(lv_event_t * e);
lv_obj_t * ui_pc2;
void ui_event_map2(lv_event_t * e);
lv_obj_t * ui_map2;
void ui_event_h7(lv_event_t * e);
lv_obj_t * ui_h7;
void ui_event_h8(lv_event_t * e);
lv_obj_t * ui_h8;
void ui_event_h9(lv_event_t * e);
lv_obj_t * ui_h9;
lv_obj_t * ui____initial_actions0;
const lv_img_dsc_t * ui_imgset_1627331012[2] = {&ui_img_86533020, &ui_img_963246739};
const lv_img_dsc_t * ui_imgset_1727632323[1] = {&ui_img_2044975918};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Button1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_OVER_BOTTOM, 200, 0, &ui_Screen2_screen_init);
    }
}
void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_OVER_BOTTOM, 200, 0, &ui_Screen3_screen_init);
    }
}
void ui_event_Button3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_OVER_BOTTOM, 200, 0, &ui_Screen4_screen_init);
    }
}
void ui_event_ranges(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_ranges_png);
    }
}
void ui_event_washing(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1799696388);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_oven(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_oven_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 300, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_bed(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_bed_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_tv(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_television_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_sink(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_sink_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_couch(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_couch_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_pc(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1996329786);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_map(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_worldwide_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_86533020);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_963246739);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image1, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_2044975918);
    }
}
void ui_event_ranges1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_ranges_png);
    }
}
void ui_event_washing1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1799696388);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_oven1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_oven_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 300, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_bed1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_bed_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_tv1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_television_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_sink1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_sink_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_couch1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_couch_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_pc1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1996329786);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_map1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_worldwide_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h4(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_86533020);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_963246739);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h6(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image2, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_2044975918);
    }
}
void ui_event_ranges2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_ranges_png);
    }
}
void ui_event_washing2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1799696388);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_oven2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_oven_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 300, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_bed2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_bed_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_tv2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_television_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_sink2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_sink_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_couch2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_couch_png);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_pc2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_1996329786);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_map2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_worldwide_png);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h7(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_86533020);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h8(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_963246739);
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_h9(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_OVER_TOP, 200, 0, &ui_Screen1_screen_init);
        _ui_image_set_property(ui_Image3, _UI_IMAGE_PROPERTY_IMAGE, & ui_img_2044975918);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui_Screen3_screen_init();
    ui_Screen4_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
