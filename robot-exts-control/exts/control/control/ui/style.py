# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
__all__ = ["julia_modeler_style"]

from omni.ui import color as cl
from omni.ui import constant as fl
from omni.ui import url
import omni.kit.app
import omni.ui as ui
import pathlib

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

ATTR_LABEL_WIDTH = 150
BLOCK_HEIGHT = 22
TAIL_WIDTH = 35
WIN_WIDTH = 400
WIN_HEIGHT = 930

# Pre-defined constants. It's possible to change them at runtime.
cl_window_bg_color = cl(0.2, 0.2, 0.2, 1.0)
cl_window_title_text = cl(.9, .9, .9, .9)
cl_collapsible_header_text = cl(.8, .8, .8, .8)
cl_collapsible_header_text_hover = cl(.95, .95, .95, 1.0)
cl_main_attr_label_text = cl(.65, .65, .65, 1.0)
cl_main_attr_label_text_hover = cl(.9, .9, .9, 1.0)
cl_multifield_label_text = cl(.65, .65, .65, 1.0)
cl_combobox_label_text = cl(.65, .65, .65, 1.0)
cl_field_bg = cl(0.18, 0.18, 0.18, 1.0)
cl_field_border = cl(1.0, 1.0, 1.0, 0.2)
cl_btn_border = cl(1.0, 1.0, 1.0, 0.4)
cl_slider_fill = cl(1.0, 1.0, 1.0, 0.3)
cl_revert_arrow_enabled = cl(.25, .5, .75, 1.0)
cl_revert_arrow_disabled = cl(.35, .35, .35, 1.0)
cl_transparent = cl(0, 0, 0, 0)

fl_main_label_attr_hspacing = 10
fl_attr_label_v_spacing = 3
fl_collapsable_group_spacing = 2
fl_outer_frame_padding = 15
fl_tail_icon_width = 15
fl_border_radius = 3
fl_border_width = 1
fl_window_title_font_size = 18
fl_field_text_font_size = 14
fl_main_label_font_size = 14
fl_multi_attr_label_font_size = 14
fl_radio_group_font_size = 14
fl_collapsable_header_font_size = 13
fl_range_text_size = 10

url_closed_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/closed.svg"
url_open_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/opened.svg"
url_revert_arrow_icon = f"{EXTENSION_FOLDER_PATH}/icons/revert_arrow.svg"
url_checkbox_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/checkbox_on.svg"
url_checkbox_off_icon = f"{EXTENSION_FOLDER_PATH}/icons/checkbox_off.svg"
url_radio_btn_on_icon = f"{EXTENSION_FOLDER_PATH}/icons/radio_btn_on.svg"
url_radio_btn_off_icon = f"{EXTENSION_FOLDER_PATH}/icons/radio_btn_off.svg"
url_diag_bg_lines_texture = f"{EXTENSION_FOLDER_PATH}/icons/diagonal_texture_screenshot.png"
# D:\DBL\Robots\robot-exts-control\exts\control\icons\diagonal_texture_screenshot.png
print("url_revert_arrow_icon: ", EXTENSION_FOLDER_PATH, "-", url_revert_arrow_icon)

# The main style dict
julia_modeler_style = {
    "Button::tool_button": {
        "background_color": cl_field_bg,
        "margin_height": 0,
        "margin_width": 6,
        "border_color": cl_btn_border,
        "border_width": fl_border_width,
        "font_size": fl_field_text_font_size,
    },
    "CollapsableFrame::group": {
        "margin_height": fl_collapsable_group_spacing,
        "background_color": cl_transparent,
    },
    # TODO: For some reason this ColorWidget style doesn't respond much, if at all (ie, border_radius, corner_flag)
    "ColorWidget": {
        "border_radius": fl_border_radius,
        "border_color": cl(0.0, 0.0, 0.0, 0.0),
    },
    "Field": {
        "background_color": cl_field_bg,
        "border_radius": fl_border_radius,
        "border_color": cl_field_border,
        "border_width": fl_border_width,
    },
    "Field::attr_field": {
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": 2,  # fl_field_text_font_size,  # Hack to allow for a smaller field border until field padding works
    },
    "Field::attribute_color": {
        "font_size": fl_field_text_font_size,
    },
    "Field::multi_attr_field": {
        "padding": 4,  # TODO: Hacky until we get padding fix
        "font_size": fl_field_text_font_size,
    },
    "Field::path_field": {
        "corner_flag": ui.CornerFlag.RIGHT,
        "font_size": fl_field_text_font_size,
    },
    "HeaderLine": {"color": cl(.5, .5, .5, .5)},
    "Image::collapsable_opened": {
        "color": cl_collapsible_header_text,
        "image_url": url_open_arrow_icon,
    },
    "Image::collapsable_opened:hovered": {
        "color": cl_collapsible_header_text_hover,
        "image_url": url_open_arrow_icon,
    },
    "Image::collapsable_closed": {
        "color": cl_collapsible_header_text,
        "image_url": url_closed_arrow_icon,
    },
    "Image::collapsable_closed:hovered": {
        "color": cl_collapsible_header_text_hover,
        "image_url": url_closed_arrow_icon,
    },
    "Image::radio_on": {"image_url": url_radio_btn_on_icon},
    "Image::radio_off": {"image_url": url_radio_btn_off_icon},
    "Image::revert_arrow": {
        "image_url": url_revert_arrow_icon,
        "color": cl_revert_arrow_enabled,
    },
    "Image::revert_arrow:disabled": {"color": cl_revert_arrow_disabled},
    "Image::checked": {"image_url": url_checkbox_on_icon},
    "Image::unchecked": {"image_url": url_checkbox_off_icon},
    "Image::slider_bg_texture": {
        "image_url": url_diag_bg_lines_texture,
        "border_radius": fl_border_radius,
        "corner_flag": ui.CornerFlag.LEFT,
    },
    "Label::attribute_name": {
        "alignment": ui.Alignment.RIGHT_TOP,
        "margin_height": fl_attr_label_v_spacing,
        "margin_width": fl_main_label_attr_hspacing,
        "color": cl_main_attr_label_text,
        "font_size": fl_main_label_font_size,
    },
    "Label::attribute_name:hovered": {"color": cl_main_attr_label_text_hover},
    "Label::collapsable_name": {"font_size": fl_collapsable_header_font_size},
    "Label::multi_attr_label": {
        "color": cl_multifield_label_text,
        "font_size": fl_multi_attr_label_font_size,
    },
    "Label::radio_group_name": {
        "font_size": fl_radio_group_font_size,
        "alignment": ui.Alignment.CENTER,
        "color": cl_main_attr_label_text,
    },
    "Label::range_text": {
        "font_size": fl_range_text_size,
    },
    "Label::window_title": {
        "font_size": fl_window_title_font_size,
        "color": cl_window_title_text,
    },
    "ScrollingFrame::window_bg": {
        "background_color": cl_window_bg_color,
        "padding": fl_outer_frame_padding,
        "border_radius": 20  # Not obvious in a window, but more visible with only a frame
    },
    "Slider::attr_slider": {
        "draw_mode": ui.SliderDrawMode.FILLED,
        "padding": 0,
        "color": cl_transparent,
        # Meant to be transparent, but completely transparent shows opaque black instead.
        "background_color": cl(0.28, 0.28, 0.28, 0.01),
        "secondary_color": cl_slider_fill,
        "border_radius": fl_border_radius,
        "corner_flag": ui.CornerFlag.LEFT,  # TODO: Not actually working yet OM-53727
    },

    # Combobox workarounds
    "Rectangle::combobox": {  # TODO: remove when ComboBox can have a border
        "background_color": cl_field_bg,
        "border_radius": fl_border_radius,
        "border_color": cl_btn_border,
        "border_width": fl_border_width,
    },
    "ComboBox::dropdown_menu": {
        "color": cl_combobox_label_text,  # label color
        "padding_height": 1.25,
        "margin": 2,
        "background_color": cl_field_bg,
        "border_radius": fl_border_radius,
        "font_size": fl_field_text_font_size,
        "secondary_color": cl_transparent,  # button background color
    },
    "Rectangle::combobox_icon_cover": {"background_color": cl_field_bg}
}
