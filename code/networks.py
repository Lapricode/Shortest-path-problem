import tkinter as tk
import tkinter.ttk as ttk
import tkinter.simpledialog as sd
import tkinter.messagebox as ms
from PIL import Image, ImageTk
import random as r
import time as t
import os
import numpy as np
import networks_algorithms as netalg
import graph_solver as grasol


class networks_app_window():
    def __init__(self, root, instance):
        self.root = root
        self.root.title(f"Window {instance+1}")
        self.root.geometry("+0+0")
        self.root.resizable(False, False)
        self.instance = instance
        # define grid_menu variables
        self.network_units_kinds = ["none", "obstacle", "vertex", "edge"]  # list of possible kinds of units on the network
        self.units_colors = [["white", "black", "#1f1f8f", "#1f9f00"], ["white", "black", "red", "yellow"]]  # colors of "none", "obstacle", "vertex" and "edge" units for the 2 color themes
        self.grid_rows = 20  # number of rows in the grid
        self.grid_columns = 20  # number of columns in the grid
        self.grid_gap = 1  # gap between the grid's units
        self.grid_units_size = 1  # size of the grid's units (in meters)
        self.grid_units_sizes_list = [1, 2, 5, 10, 20, 50, 100]  # list of possible sizes of the grid's units (in meters)
        self.normal_velocity = 10  # normal velocity of the ambulance (in km/h)
        self.normal_velocities_list = [10, 20, 30, 40, 50, 60]  # list of possible normal velocities of the ambulance (in km/h)
        self.grid_rows_list = [5, 10, 20, 30, 40, 50, 100]  # list of possible number of rows in the grid
        self.grid_columns_list = [5, 10, 20, 30, 40, 50, 100]  # list of possible number of columns in the grid
        self.grid_gaps_list = [0, 1, 2, 5, 10]  # list of possible gaps between the grid's units
        self.grid_color_theme = 0  # 0 = blue/green, 1 = red/yellow
        self.grid_color_themes_list = ["blue/\ngreen", "red/\nyellow"]  # list of possible color themes of the grid
        # define graph_info_menu variables
        self.vertices_num = 0  # number of vertices on the network
        self.edges_num = 0  # number of edges on the network
        self.vertex_edge_choice = "vertex"  # if "vertex" then choose a vertex, if "edge" then choose an edge
        self.vertex_edge_choices_list = ["vertex", "edge"]  # list of possible values of vertex_edge_choice
        self.highlight_vertices_edges_mode = "start"  # if "start" then point to a vertex/edge on the network, if "stop" then stop pointing
        self.highlight_vertices_graph_edges_modes_list = ["start", "stop"]  # list of possible values of highlight_vertices_edges_mode
        # define create_graph_menu variables
        self.graph_vertices = []  # list of graph vertices
        self.graph_edges = {}  # dictionary of graph edges
        self.graph_edges_directionalities = {}  # dictionary of edges directions
        self.direction_arrows_list = [["←", "→", "↔"], ["↓", "↑", "↕"]]  # list of possible direction arrows
        self.add_edge_control = "first_vertex"  # if "first_vertex" then choose the first vertex of the edge to be added manually, else the second vertex
        self.add_edge_first_vertex = None  # this is the first vertex of the edge to be added manually
        self.auto_vertices_num = 10  # number of vertices to be added automatically
        self.auto_vertices_num_list = [3, 5, 10, 20, 30, 50, 100]  # list of possible values of auto_vertices_num
        self.random_vertices_edges_choose = "vertices"  # if "vertices" then choose random vertices, if "edges" then choose random edges
        self.random_vertices_edges_choose_list = ["vertices", "edges"]  # list of possible values of random_vertices_edges_choose
        self.create_graph_manually_mode = "start"  # if "start" then create the graph manually, if "stop" then stop creating the graph manually
        self.create_graph_manually_modes_list = ["start", "stop"]  # list of possible values of create_graph_manually_mode
        self.time_weights_values_list = [2, 3, 4, 5, 6, 7, 8, 9, 10]  # list of possible time weights values
        self.time_weights_names_list = [image[:-4] for image in os.listdir(os.getcwd() + "/images/time_weights_images/") if image.endswith(".png")]  # list of possible time weight images names
        self.time_weight_name_current = self.time_weights_names_list[0]  # the current time weight name
        self.time_weights_values = dict(zip(self.time_weights_names_list, [r.choice(self.time_weights_values_list) for k in range(len(self.time_weights_names_list))]))  # dictionary of time weights
        self.time_weights_values["default"] = 1  # the default time weight
        self.add_time_weights_mode = "start"  # if "start" then add time weights, if "stop" then stop adding time weights
        self.add_time_weights_modes_list = ["start", "stop"]  # list of possible values of add_time_weights_mode
        self.obstacles_on_grid = 0  # if 0 there are not obstacles on the grid, if 1 there are obstacles on the grid
        # define solve_problem_menu variables
        self.add_target_vertices_mode = "start"  # if "start" then add target vertices, if "stop" then stop adding target vertices
        self.add_target_vertices_modes_list = ["start", "stop"]  # list of possible values of add_target_vertices_mode
        self.target_vertices_names_list = [image[:-4] for image in os.listdir(os.getcwd() + "/images/target_vertices_images/") if image.endswith(".png")]  # list of possible target vertices images names
        self.target_vertex_name_current = self.target_vertices_names_list[0]  # the current target vertex name
        self.target_vertices = {}  # dictionary of target vertices already added on the network
        self.optimization_type = "length"  # the optimization problem to be solved
        self.optimization_types_list = ["length", "time"]  # list of possible optimization problems
        self.showing_path_1 = False  # if True then show the path from the ambulance to the patient
        self.showing_path_2 = False  # if True then show the path from the patient to the hospital
        self.path_1_color = "white"  # the color of the path from the ambulance to the patient
        self.path_2_color = "salmon"  # the color of the path from the patient to the hospital
        self.shortest_path_ambulance_patient_units = []  # the graph units containted in the shortest path from the ambulance to the patient
        self.shortest_path_patient_hospital_units = []  # the graph units containted in the shortest path from the patient to the hospital
        self.entered_solve_mode = False  # if True then the user has entered the solve mode
        self.sens_analysis_enabled = False  # if True then the sensitivity analysis can be applied, if False then it can not be applied
        self.IP_solve_mode = False  # if True then the user has entered the IP solve mode
        self.show_mod_sol = "model"  # if "model" then show problem's model, if "solution" then show problem's solution
        self.show_mod_sol_list = ["model", "solution"]  # list of possible values
        # create the network_place (the network space), the menus_background (the menus space) and all the submenus (grid_menu, ...)
        if "saved_graphs" not in os.listdir(os.getcwd()):
            os.mkdir(os.getcwd() + "/saved_graphs")
        self.create_network_place_menus()

    def create_network_place_menus(self):
        # create the network_place (the network space), the menus_background (the menus space) and all the submenus (grid_menu, ...)
        # define the network_place's width, height, edges and corners
        self.network_place_height = self.root.winfo_screenheight() * 3 / 4
        self.network_place_width = self.network_place_height
        self.network_place = tk.Canvas(self.root, width = self.network_place_width, height = self.network_place_height, bg = "black")
        self.network_place.grid(row = 1, column = 1, sticky = tk.NSEW)
        borders_width = 60
        network_place_edges_color = "cyan"
        network_place_corners_color = "lime"
        self.network_place_up_edge = tk.Frame(self.root, width = self.network_place_width, height = borders_width, bg = network_place_edges_color)
        self.network_place_down_edge = tk.Frame(self.root, width = self.network_place_width, height = borders_width, bg = network_place_edges_color)
        self.network_place_left_edge = tk.Frame(self.root, width = borders_width, height = self.network_place_height, bg = network_place_edges_color)
        self.network_place_right_edge = tk.Frame(self.root, width = borders_width, height = self.network_place_height, bg = network_place_edges_color)
        self.network_place_up_left_corner = tk.Frame(self.root, width = borders_width, height = borders_width, bg = network_place_corners_color)
        self.network_place_up_right_corner = tk.Frame(self.root, width = borders_width, height = borders_width, bg = network_place_corners_color)
        self.network_place_down_left_corner = tk.Frame(self.root, width = borders_width, height = borders_width, bg = network_place_corners_color)
        self.network_place_down_right_corner = tk.Frame(self.root, width = borders_width, height = borders_width, bg = network_place_corners_color)
        self.network_place_up_edge.grid(row = 0, column = 1, sticky = tk.NSEW)
        self.network_place_down_edge.grid(row = 2, column = 1, sticky = tk.NSEW)
        self.network_place_left_edge.grid(row = 1, column = 0, sticky = tk.NSEW)
        self.network_place_right_edge.grid(row = 1, column = 2, sticky = tk.NSEW)
        self.network_place_up_left_corner.grid(row = 0, column = 0, sticky = tk.NSEW)
        self.network_place_up_right_corner.grid(row = 0, column = 2, sticky = tk.NSEW)
        self.network_place_down_left_corner.grid(row = 2, column = 0, sticky = tk.NSEW)
        self.network_place_down_right_corner.grid(row = 2, column = 2, sticky = tk.NSEW)
        # up edge options
        menu_label(self.network_place_up_edge, "City's Roads Network", f"Arial 20 bold", "black", network_place_edges_color, 1/2 * self.network_place_width, 1/2 * borders_width)
        self.graph_state_indicator = menu_label(self.network_place_up_edge, "Program started\nrunning...", f"Arial 12 bold", "red", network_place_edges_color, 7/8 * self.network_place_width, 1/2 * borders_width).label
        # right edge options
        right_edge_rows = 8
        sm_y, spf_y, sps_y, esm_y = 3, 4, 5, 6
        menu_label(self.network_place_right_edge, "Solve\nmode:", f"Arial 12 bold", "black", network_place_edges_color, 1/2 * borders_width, sm_y * self.network_place_width / (right_edge_rows + 1))
        self.show_path_1_button = menu_button(self.network_place_right_edge, "path\n1", f"Calibri 10 bold", "brown", network_place_edges_color, 1/2 * borders_width, spf_y * self.network_place_width / (right_edge_rows + 1), self.show_path_1).button
        self.show_path_2_button = menu_button(self.network_place_right_edge, "path\n2", f"Calibri 10 bold", "brown", network_place_edges_color, 1/2 * borders_width, sps_y * self.network_place_width / (right_edge_rows + 1), self.show_path_2).button
        self.exit_solve_mode_button = menu_button(self.network_place_right_edge, "Exit\nsolve\nmode", f"Calibri 10 bold", "red", network_place_edges_color, 1/2 * borders_width, esm_y * self.network_place_width / (right_edge_rows + 1), self.exit_solve_mode).button
        # corners options
        menu_label(self.network_place_down_left_corner, "Gap: ", f"Arial 15 bold", "black", network_place_corners_color, 1/2 * borders_width, 1/4 * borders_width)
        self.grid_gap_button = menu_button(self.network_place_down_left_corner, self.grid_gap, f"Calibri 15 bold", "brown", network_place_corners_color, 1/2 * borders_width, 3/4 * borders_width, self.change_parameters_actions).button
        self.grid_color_theme_button = menu_button(self.network_place_down_right_corner, self.grid_color_themes_list[self.grid_color_theme], f"Calibri 10 bold", "black", network_place_corners_color, 1/2 * borders_width, 1/2 * borders_width, self.change_parameters_actions).button
        menu_label(self.network_place_up_left_corner, "Unit\nsize (m): ", f"Arial 10 bold", "black", network_place_corners_color, 1/2 * borders_width, 1/4 * borders_width)
        self.grid_units_size_button = menu_button(self.network_place_up_left_corner, self.grid_units_size, f"Calibri 15 bold", "brown", network_place_corners_color, 1/2 * borders_width, 3/4 * borders_width, self.change_parameters_actions).button
        menu_label(self.network_place_up_right_corner, "Normal vel\n(km/h): ", f"Arial 8 bold", "black", network_place_corners_color, 1/2 * borders_width, 1/4 * borders_width)
        self.normal_velocity_button = menu_button(self.network_place_up_right_corner, self.normal_velocity, f"Calibri 15 bold", "brown", network_place_corners_color, 1/2 * borders_width, 3/4 * borders_width, self.change_parameters_actions).button
        down_edge_font = 12
        down_edge_rows = 2
        down_edge_columns = 5
        gr_ord_x, gc_ord_x, gl_ord_x, gw_ord_x, ven_ord_x, edn_ord_x, tc_ord_x = 0.5, 0.5, 1.9, 1.9, 3.3, 3.3, 4.8
        gr_ord_y, gc_ord_y, gl_ord_y, gw_ord_y, ven_ord_y, edn_ord_y, tc_ord_y = 1, 2, 1, 2, 1, 2, 1.5
        menu_label(self.network_place_down_edge, "Rows:", f"Arial {down_edge_font} bold", "black", network_place_edges_color, gr_ord_x * self.network_place_width / (down_edge_columns + 1), gr_ord_y * borders_width / (down_edge_rows + 1))
        self.grid_rows_indicator = menu_label(self.network_place_down_edge, self.grid_rows, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (gr_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1), gr_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Columns:", f"Arial {down_edge_font} bold", "black", network_place_edges_color, gc_ord_x * self.network_place_width / (down_edge_columns + 1), gc_ord_y * borders_width / (down_edge_rows + 1))
        self.grid_columns_indicator = menu_label(self.network_place_down_edge, self.grid_columns, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (gc_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1), gc_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Length (m):", f"Arial {down_edge_font} bold", "black", network_place_edges_color, gl_ord_x * self.network_place_width / (down_edge_columns + 1), gl_ord_y * borders_width / (down_edge_rows + 1))
        self.grid_length_indicator = menu_label(self.network_place_down_edge, self.grid_units_size * self.grid_rows, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (gl_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1) + 10, gl_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Width (m):", f"Arial {down_edge_font} bold", "black", network_place_edges_color, gw_ord_x * self.network_place_width / (down_edge_columns + 1), gw_ord_y * borders_width / (down_edge_rows + 1))
        self.grid_width_indicator = menu_label(self.network_place_down_edge, self.grid_units_size * self.grid_columns, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (gw_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1) + 10, gw_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Vertices #:", f"Arial {down_edge_font} bold", "black", network_place_edges_color, ven_ord_x * self.network_place_width / (down_edge_columns + 1), ven_ord_y * borders_width / (down_edge_rows + 1))
        self.vertices_num_indicator = menu_label(self.network_place_down_edge, self.vertices_num, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (ven_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1), ven_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Edges #:", f"Arial {down_edge_font} bold", "black", network_place_edges_color, edn_ord_x * self.network_place_width / (down_edge_columns + 1), edn_ord_y * borders_width / (down_edge_rows + 1))
        self.edges_num_indicator = menu_label(self.network_place_down_edge, self.edges_num, f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (edn_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1), edn_ord_y * borders_width / (down_edge_rows + 1)).label
        menu_label(self.network_place_down_edge, "Normal time cost\n(n.t.c.) per unit (sec):", f"Arial {down_edge_font} bold", "black", network_place_edges_color, tc_ord_x * self.network_place_width / (down_edge_columns + 1), tc_ord_y * borders_width / (down_edge_rows + 1))
        self.normal_time_cost_indicator = menu_label(self.network_place_down_edge, round(3.6 * self.grid_units_size / self.normal_velocity, 3), f"Calibri {down_edge_font} bold", "brown", network_place_edges_color, (tc_ord_x+1/2) * self.network_place_width / (down_edge_columns + 1) + 45, tc_ord_y * borders_width / (down_edge_rows + 1)).label

        # define the menus' backgrounds
        menus_background_width = 12 * borders_width
        menus_background_height = self.network_place_height + 2 * borders_width
        self.menus_background_1 = tk.Frame(self.root, width = menus_background_width, height = menus_background_height, bg = "black")
        self.menus_background_1.grid(row = 0, rowspan = 3, column = 3, sticky = tk.NSEW)
        self.menus_background_2 = tk.Frame(self.root, width = menus_background_width, height = menus_background_height, bg = "black")
        self.menus_background_2.grid(row = 0, rowspan = 3, column = 4, sticky = tk.NSEW)
        
        # grid_menu for network's grid features
        self.grid_menu_rows = 4
        grid_menu_width = 1/2 * menus_background_width
        grid_menu_height = 2/8 * menus_background_height
        grid_menu_font = 16
        grid_menu_bg_color = "black"
        self.grid_menu = tk.Frame(self.menus_background_1, width = grid_menu_width, height = grid_menu_height, bg = grid_menu_bg_color, highlightbackground = "red", highlightthickness = 5)
        self.grid_menu.grid(row = 0, column = 0, sticky = tk.NSEW)
        ng_ord, r_ord, c_ord, cg_ord = 1, 2, 3, 4
        ng_x, cg_x, sg_x = 1/2, 1/2, 1/2
        r_x, c_x = 1/3, 1/3
        menu_label(self.grid_menu, "Network's Grid:", f"Arial {grid_menu_font + 5} bold underline", "gold", grid_menu_bg_color, ng_x * grid_menu_width, ng_ord * grid_menu_height / (self.grid_menu_rows + 1))
        menu_label(self.grid_menu, "Rows:", f"Arial {grid_menu_font} bold", "#00ffff", grid_menu_bg_color, r_x * grid_menu_width, r_ord * grid_menu_height / (self.grid_menu_rows + 1))
        self.grid_rows_button = menu_button(self.grid_menu, self.grid_rows, f"Calibri {grid_menu_font + 5} bold", "white", grid_menu_bg_color, (1 - r_x) * grid_menu_width, r_ord * grid_menu_height / (self.grid_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.grid_menu, "Columns: ", f"Arial {grid_menu_font} bold", "#00ffff", grid_menu_bg_color, c_x * grid_menu_width, c_ord * grid_menu_height / (self.grid_menu_rows + 1))
        self.grid_columns_button = menu_button(self.grid_menu, self.grid_columns, f"Calibri {grid_menu_font + 5} bold", "white", grid_menu_bg_color, (1 - c_x) * grid_menu_width, c_ord * grid_menu_height / (self.grid_menu_rows + 1), self.change_parameters_actions).button
        menu_button(self.grid_menu, "Create new grid", f"Calibri {grid_menu_font + 5} bold", "white", grid_menu_bg_color, cg_x * grid_menu_width, cg_ord * grid_menu_height / (self.grid_menu_rows + 1), self.new_network_grid).button
        
        # create_graph_menu for creating and designing the network's graph
        self.create_graph_menu_rows = 13
        create_graph_menu_width = 1/2 * menus_background_width
        create_graph_menu_height = 6/8 * menus_background_height
        create_graph_menu_font = 16
        create_graph_menu_bg_color = "black"
        self.create_graph_menu = tk.Frame(self.menus_background_1, width = create_graph_menu_width, height = create_graph_menu_height, bg = create_graph_menu_bg_color, highlightbackground = "red", highlightthickness = 5)
        self.create_graph_menu.grid(row = 1, column = 0, sticky = tk.NSEW)
        cdn_ord, ac_ord, vn_ord, rvec_ord, mc_ord, mcb_ord, atw_ord, wv_ord, ed_ord, sg_ord, lg_ord = 1, 2, 3, 4, 5, 6, 7.5, 9, 10.7, 12, 13
        cdn_x, ac_x, mc_x, sg_x = 1/2, 1/2, 1/2, 1/2
        vn_x, lg_x, rvec_x, mcb_x, atw_x, wv_x, ed_x = 1/3, 1/3, 1/3, 1/3, 1/3, 1/3, 1/3
        menu_label(self.create_graph_menu, "Create Graph:", f"Arial {create_graph_menu_font + 5} bold underline", "gold", create_graph_menu_bg_color, cdn_x * create_graph_menu_width, cdn_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        menu_label(self.create_graph_menu, "Auto (random):", f"Arial {create_graph_menu_font + 3} bold italic underline", "pink", create_graph_menu_bg_color, ac_x * create_graph_menu_width, ac_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        menu_label(self.create_graph_menu, "# of Vertices:", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, vn_x * create_graph_menu_width, vn_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.auto_vertices_num_button = menu_button(self.create_graph_menu, self.auto_vertices_num, f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, (1 - vn_x) * create_graph_menu_width, vn_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        self.random_vertices_edges_choose_button = menu_button(self.create_graph_menu, self.random_vertices_edges_choose, f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, rvec_x * create_graph_menu_width, rvec_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.create_graph_menu, ":", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, 1/2 * create_graph_menu_width, rvec_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.random_add_button = menu_button(self.create_graph_menu, "add", f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, (1 - rvec_x) * create_graph_menu_width, rvec_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.generate_random_vertices_edges).button
        menu_label(self.create_graph_menu, "Manual (control):", f"Arial {create_graph_menu_font + 3} bold italic underline", "pink", create_graph_menu_bg_color, mc_x * create_graph_menu_width, mc_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        menu_label(self.create_graph_menu, "Build:", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, mcb_x * create_graph_menu_width, mcb_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.create_graph_manually_button = menu_button(self.create_graph_menu, self.create_graph_manually_mode, f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, (1 - mcb_x) * create_graph_menu_width, mcb_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.create_graph_menu, "Add time\nweights:", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, atw_x * create_graph_menu_width, atw_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.time_weights_images_canvas_width = 50
        self.time_weights_images_canvas_height = 50
        self.time_weights_images_canvas = tk.Canvas(self.create_graph_menu, width = self.time_weights_images_canvas_width, height = self.time_weights_images_canvas_height, bg = self.units_colors[0][3])
        self.time_weights_images_canvas.place(x = (1 - 1.2 * atw_x) * create_graph_menu_width, y = atw_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), anchor = "center")
        self.time_weights_next_image_button = menu_button(self.create_graph_menu, "➤", f"Calibri {create_graph_menu_font - 2} bold", "white", create_graph_menu_bg_color, (1 - 0.7 * atw_x) * create_graph_menu_width, 0.95 * atw_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        self.add_time_weights_button = menu_button(self.create_graph_menu, self.add_time_weights_mode, f"Calibri {create_graph_menu_font - 2} bold", "white", create_graph_menu_bg_color, (1 - 0.7 * atw_x) * create_graph_menu_width, 1.05 * atw_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.create_graph_menu, "Time weights\nvalues (x n.t.c.):", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, wv_x * create_graph_menu_width, wv_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.time_weights_values_button = menu_button(self.create_graph_menu, self.time_weights_values[self.time_weight_name_current], f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, (1 - wv_x) * create_graph_menu_width, wv_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.create_graph_menu, "Edges\ndirectionality:", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, ed_x * create_graph_menu_width, ed_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.edges_directionality_choose = ttk.Combobox(self.create_graph_menu, font = 'Calibri 14', state = 'readonly', width = 3, values = [])
        self.edges_directionality_choose.place(x = (1 - ed_x) * create_graph_menu_width, y = (ed_ord - 0.5) * create_graph_menu_height / (self.create_graph_menu_rows + 1), anchor = "center")
        self.edges_directionality_choose.bind("<<ComboboxSelected>>", self.highlight_vertices_edges)
        self.edges_directionality_change_button = menu_button(self.create_graph_menu, self.direction_arrows_list[0][2], f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, (1 - ed_x/2) * create_graph_menu_width, (ed_ord - 0.5) * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.change_parameters_actions).button
        self.make_graph_undirected_button = menu_button(self.create_graph_menu, "undirected", f"Calibri {create_graph_menu_font - 2} bold", "white", create_graph_menu_bg_color, (1 - ed_x*3/4) * create_graph_menu_width, (ed_ord + 0.5) * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.make_graph_undirected).button
        self.save_graph_button = menu_button(self.create_graph_menu, "Save graph", f"Calibri {create_graph_menu_font + 5} bold", "white", create_graph_menu_bg_color, sg_x * create_graph_menu_width, sg_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), self.save_graph).button
        menu_label(self.create_graph_menu, "Load graph:", f"Arial {create_graph_menu_font} bold", "#00ffff", create_graph_menu_bg_color, lg_x * create_graph_menu_width, lg_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1))
        self.saved_graph_choose = ttk.Combobox(self.create_graph_menu, font = 'Calibri 12', state = 'readonly', width = 10, values = [])
        self.saved_graph_choose.place(x = (1 - lg_x) * create_graph_menu_width, y = lg_ord * create_graph_menu_height / (self.create_graph_menu_rows + 1), anchor = "center")
        self.saved_graph_choose.bind("<<ComboboxSelected>>", self.load_graph)
        self.load_time_weight_image(self.time_weight_name_current)
        self.get_saved_graphs()

        # graph_info_menu for network's graph information
        self.graph_info_menu_rows = 8
        graph_info_menu_width = 1/2 * menus_background_width
        graph_info_menu_height = 5/9 * menus_background_height
        graph_info_menu_font = 16
        graph_info_menu_bg_color = "black"
        self.graph_info_menu = tk.Frame(self.menus_background_2, width = graph_info_menu_width, height = graph_info_menu_height, bg = graph_info_menu_bg_color, highlightbackground = "red", highlightthickness = 5)
        self.graph_info_menu.grid(row = 0, column = 1, sticky = tk.NSEW)
        gi_ord, guc_ord, n_ord, h_ord, di_ord, ib_ord = 1, 2, 3, 4, 5, 7
        gi_x, ib_x, di_x = 1/2, 1/2, 1/2
        n_x, h_x, guc_x = 1/3, 1/3, 1/3
        menu_label(self.graph_info_menu, "Graph Information:", f"Arial {graph_info_menu_font + 5} bold underline", "gold", graph_info_menu_bg_color, gi_x * graph_info_menu_width, gi_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1))
        menu_label(self.graph_info_menu, "Grid unit\nchoice:", f"Arial {graph_info_menu_font} bold", "#00ffff", graph_info_menu_bg_color, guc_x * graph_info_menu_width, guc_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1))
        self.vertex_edge_choice_button = menu_button(self.graph_info_menu, self.vertex_edge_choice, f"Calibri {graph_info_menu_font + 5} bold", "white", graph_info_menu_bg_color, (1 - guc_x) * graph_info_menu_width, guc_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.graph_info_menu, "Number:", f"Arial {graph_info_menu_font} bold", "#00ffff", graph_info_menu_bg_color, n_x * graph_info_menu_width, n_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1))
        self.vertices_edges_choose = ttk.Combobox(self.graph_info_menu, font = 'Calibri 20', state = 'readonly', width = 5, values = [])
        self.vertices_edges_choose.place(x = (1 - n_x) * graph_info_menu_width, y = n_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1), anchor = "center")
        self.vertices_edges_choose.bind("<<ComboboxSelected>>", self.highlight_vertices_edges)
        menu_label(self.graph_info_menu, "Highlight:", f"Arial {graph_info_menu_font} bold", "#00ffff", graph_info_menu_bg_color, h_x * graph_info_menu_width, h_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1))
        self.highlight_vertices_edges_button = menu_button(self.graph_info_menu, self.highlight_vertices_edges_mode, f"Calibri {graph_info_menu_font + 5} bold", "white", graph_info_menu_bg_color, (1 - h_x) * graph_info_menu_width, h_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.graph_info_menu, "Display information: ", f"Arial {graph_info_menu_font} bold", "#00ffff", graph_info_menu_bg_color, di_x * graph_info_menu_width, di_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1))
        self.information_box = tk.Text(self.graph_info_menu, font = 'Calibri 10 bold', width = 40, height = 10, wrap = "word")
        self.information_box.place(x = ib_x * graph_info_menu_width, y = ib_ord * graph_info_menu_height / (self.graph_info_menu_rows + 1), anchor = "center")

        # solve_problem_menu for solving the optimization problem
        self.solve_problem_menu_rows = 7
        solve_problem_menu_width = 1/2 * menus_background_width
        solve_problem_menu_height = 4/9 * menus_background_height
        solve_problem_menu_font = 16
        solve_problem_menu_bg_color = "black"
        self.solve_problem_menu = tk.Frame(self.menus_background_2, width = solve_problem_menu_width, height = solve_problem_menu_height, bg = solve_problem_menu_bg_color, highlightbackground = "red", highlightthickness = 5)
        self.solve_problem_menu.grid(row = 1, column = 1, sticky = tk.NSEW)
        sg_ord, cc_ord, atv_ord, opc_ord, sop_ord, asa_ord = 1, 2, 3.3, 4.7, 6, 7
        sg_x, asa_x, cc_x = 1/2, 1/2, 1/2
        opc_x, sop_x, atv_x = 1/3, 1/3, 1/3
        menu_label(self.solve_problem_menu, "Solve Graph:", f"Arial {solve_problem_menu_font + 5} bold underline", "gold", solve_problem_menu_bg_color, sg_x * solve_problem_menu_width, sg_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1))
        self.check_graph_connectivity_button = menu_button(self.solve_problem_menu, "Check connectivity", f"Calibri {solve_problem_menu_font + 5} bold", "white", solve_problem_menu_bg_color, cc_x * solve_problem_menu_width, cc_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), self.check_graph_connectivity).button
        menu_label(self.solve_problem_menu, "Add target\nvertices:", f"Arial {solve_problem_menu_font} bold", "#00ffff", solve_problem_menu_bg_color, atv_x * solve_problem_menu_width, atv_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1))
        self.target_vertices_images_canvas_width = 50
        self.target_vertices_images_canvas_height = 50
        self.target_vertices_images_canvas = tk.Canvas(self.solve_problem_menu, width = self.target_vertices_images_canvas_width, height = self.target_vertices_images_canvas_height, bg = self.units_colors[0][2])
        self.target_vertices_images_canvas.place(x = (1 - 1.2 * atv_x) * solve_problem_menu_width, y = atv_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), anchor = "center")
        self.target_vertices_next_image_button = menu_button(self.solve_problem_menu, "➤", f"Calibri {solve_problem_menu_font - 2} bold", "white", solve_problem_menu_bg_color, (1 - 0.7 * atv_x) * solve_problem_menu_width, 0.90 * atv_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), self.change_parameters_actions).button
        self.add_target_vertices_button = menu_button(self.solve_problem_menu, self.add_target_vertices_mode, f"Calibri {solve_problem_menu_font - 2} bold", "white", solve_problem_menu_bg_color, (1 - 0.7 * atv_x) * solve_problem_menu_width, 1.10 * atv_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.solve_problem_menu, "Optimization\nProblem:", f"Arial {solve_problem_menu_font} bold", "#00ffff", solve_problem_menu_bg_color, opc_x * solve_problem_menu_width, opc_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1))
        self.opt_problem_choose_button = menu_button(self.solve_problem_menu, self.optimization_type, f"Calibri {solve_problem_menu_font + 5} bold", "white", solve_problem_menu_bg_color, (1 - opc_x) * solve_problem_menu_width, opc_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), self.change_parameters_actions).button
        menu_label(self.solve_problem_menu, "Solve:", f"Arial {solve_problem_menu_font} bold", "#00ffff", solve_problem_menu_bg_color, sop_x * solve_problem_menu_width, sop_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1))
        self.solve_opt_problem_with_IP_button = menu_button(self.solve_problem_menu, "LP / IP", f"Calibri {solve_problem_menu_font - 3} bold", "white", solve_problem_menu_bg_color, (1 - sop_x) * solve_problem_menu_width, sop_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1) - 12, self.find_shortest_path_with_IP).button
        self.solve_opt_problem_with_Astar_button = menu_button(self.solve_problem_menu, "A* (A star)", f"Calibri {solve_problem_menu_font - 3} bold", "white", solve_problem_menu_bg_color, (1 - sop_x) * solve_problem_menu_width, sop_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1) + 12, self.find_shortest_path_with_Astar).button
        self.apply_sensitivity_analysis_button = menu_button(self.solve_problem_menu, "Sensitivity analysis", f"Calibri {solve_problem_menu_font + 5} bold", "white", solve_problem_menu_bg_color, (1 - asa_x) * solve_problem_menu_width, asa_ord * solve_problem_menu_height / (self.solve_problem_menu_rows + 1), self.apply_sensitivity_analysis).button
        self.load_target_vertex_image(self.target_vertex_name_current)

        # create a new empty network grid (all the graph units are "none")
        self.new_network_grid()

    def new_network_grid(self, event = None):
        self.network_place.delete("all")
        self.grid_rows_indicator.configure(text = self.grid_rows)
        self.grid_columns_indicator.configure(text = self.grid_columns)
        self.grid_length_indicator.configure(text = self.grid_units_size * self.grid_rows)
        self.grid_width_indicator.configure(text = self.grid_units_size * self.grid_columns)
        self.graph_vertices = []
        self.graph_edges = {}
        self.graph_edges_directionalities = {}
        self.update_vertices_edges_info_choose_list()
        self.target_vertices = {}
        self.network_units_list = []
        for i in range(self.grid_rows):
            self.network_units_list.append([])
            for j in range(self.grid_columns):
                self.network_units_list[-1].append(network_unit(self.instance, self.network_place, 0, i * self.grid_columns + j, i, j, self.network_place_width / self.grid_columns, self.network_place_height / self.grid_rows, self.grid_gap, "white"))
                self.network_units_list[-1][-1].set_unit_on_grid()
        self.write_information_box("Info", "blue", f"New network grid created with {self.grid_rows} rows and {self.grid_columns} columns.")

    def same_network_grid(self, event = None):
        for graph_row in self.network_units_list:
            for graph_unit in graph_row:
                graph_unit.border = self.grid_gap
                self.network_place.itemconfigure(graph_unit.unit, width = graph_unit.border)
                graph_unit.change_unit_kind_color(graph_unit.kind)

    def generate_random_vertices_edges(self, event = None):
        if self.random_vertices_edges_choose == "vertices":
            self.generate_random_vertices()
        elif self.random_vertices_edges_choose == "edges":
            self.generate_random_edges()

    # create random graph vertices
    def generate_random_vertices(self, event = None):
        if len(self.network_units_list)*len(self.network_units_list[0])/2 < self.auto_vertices_num:
            self.write_information_box("Warning", "red", f"There are not enouph graph units for this number of vertices ({self.auto_vertices_num}) or it is not possible for all the vertices to be at least 2 units of manhattan distance away from each other!")
        else:
            self.graph_state_indicator.configure(text = "Generating\nrandom vertices ...")
            self.new_network_grid()
            self.obstacles_on_grid = 0
            timeout_counter = 0
            while len(self.graph_vertices) < self.auto_vertices_num and timeout_counter < 1000 * self.auto_vertices_num:
                timeout_counter += 1
                i = r.randint(0, len(self.network_units_list)-1)
                j = r.randint(0, len(self.network_units_list[0])-1)
                if (i, j) not in self.graph_vertices and (self.get_nearest_vertex((i, j)))[1] > 1:
                    self.graph_vertices.append((i, j))
            if len(self.graph_vertices) < self.auto_vertices_num:
                self.write_information_box("Warning", "red", f"Could not place the vertices on the graph before the timeout, in order for all of them to be at least 2 units of manhattan distance apart!")
                self.graph_vertices = []
            else:
                for (i, j) in self.graph_vertices:
                    self.network_units_list[i][j].change_unit_kind_color(2)
                self.write_information_box("Info", "blue", f"Randomly placed {len(self.graph_vertices)} vertices on the graph.")
            self.update_vertices_edges_info_choose_list()
        
    # create random edges connecting the vertices of the graph
    def generate_random_edges(self, event = None):
        for graph_row in self.network_units_list:  # change all the "edge" graph units to "none"
            for graph_unit in graph_row:
                if graph_unit.kind == 3:
                    graph_unit.change_unit_kind_color(self.obstacles_on_grid)
                    self.remove_unit_images_directionality((graph_unit.row, graph_unit.column))
        if len(self.graph_vertices) < 2:
            self.write_information_box("Warning", "red", "There are not enouph graph vertices in order to generate edges!")
        else:
            self.graph_edges = {}
            self.graph_edges_directionalities = {}
            self.graph_state_indicator.configure(text = "Generating\nrandom edges ...")
            if self.obstacles_on_grid == 1:  # delete the obstacles (if there are any) before generating the edges
                self.add_delete_obstacles()
            for vertex1 in self.graph_vertices:
                for vertex2 in self.graph_vertices:
                    # generate a new edge
                    self.create_new_edge(vertex1, vertex2, r.randint(0, 1))
            self.update_vertices_edges_info_choose_list()
            self.write_information_box("Info", "blue", f"Automatically generated random directed graph with {len(self.graph_vertices)} vertices and {len(self.graph_edges)} edges.")
            self.graph_state_indicator.configure(text = "Random graph\nis completed.")
    
    def create_new_edge(self, vertex1, vertex2, random_edge):
        vertices_pair = tuple(netalg.sort_vertices_by_manhattan_distance([vertex1, vertex2], (0, 0)))
        if vertex1 != vertex2 and vertices_pair not in self.graph_edges.keys():  # not allowing multigraphs
            edge = []
            x_dist = vertex2[1] - vertex1[1]
            y_dist = vertex2[0] - vertex1[0]
            last_i = 1
            for i in range(1, abs([x_dist, y_dist][random_edge]) + 1):
                edge.append((vertex1[0] + random_edge * np.sign(y_dist)*i, vertex1[1] + (1-random_edge) * np.sign(x_dist)*i))
                last_i = i
            for j in range(1, abs([y_dist, x_dist][random_edge]) + 1):
                edge.append((vertex1[0] + np.sign(y_dist)*[j, last_i][random_edge], vertex1[1] + np.sign(x_dist)*[last_i, j][random_edge]))
            if vertex2 in edge:
                edge.remove(vertex2)
            if vertices_pair == (vertex2, vertex1):  # if the vertices are in the opposite order, reverse the edge
                edge.reverse()
            # check if the new edge intersects with any other edge or if it contains any vertex and if so do not add it to the edges list
            not_edges_intersect = True
            for edge2 in self.graph_edges.values():
                if list(set(edge).intersection(edge2)) != []:
                    not_edges_intersect = False
                    break
            if not_edges_intersect and list(set(edge).intersection(self.graph_vertices)) == []:
                self.graph_edges[vertices_pair] = edge
                choose_directions_list = [True, False].index(vertices_pair[0][0] == edge[0][0])
                self.graph_edges_directionalities[edge[0]] = r.choice(self.direction_arrows_list[choose_directions_list])
                self.network_units_list[edge[0][0]][edge[0][1]].add_unit_edge_directionality(self.graph_edges_directionalities[edge[0]])
                for edge_unit in edge:
                    # t.sleep(0.01)
                    self.network_place.update()
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
    
    def convert_edge_directionality(self, edge_number):
        edge_first_vertex = list(self.graph_edges.keys())[edge_number][0]
        edge_first_unit = list(self.graph_edges_directionalities.keys())[edge_number]
        edge_directionality = self.graph_edges_directionalities[edge_first_unit]
        arrow_index = self.direction_arrows_list[abs(edge_first_vertex[0] - edge_first_unit[0])].index(edge_directionality)
        edge_unit_cor_check = 1-abs(edge_first_vertex[0] - edge_first_unit[0])
        directionality_index = edge_first_vertex[edge_unit_cor_check] - edge_first_unit[edge_unit_cor_check]
        if arrow_index == 2:
            converted_edge = self.direction_arrows_list[0][2]
        elif arrow_index == 0 or arrow_index == 1:
            converted_edge = self.direction_arrows_list[0][int(abs(arrow_index - (1+(2*edge_unit_cor_check-1)*directionality_index)/2))]
        return converted_edge
    
    def make_graph_undirected(self, event = None):
        for vertex in list(self.graph_edges_directionalities.keys()):
            if self.graph_edges_directionalities[vertex] in self.direction_arrows_list[0]:
                new_edge_direction = "↔"
            elif self.graph_edges_directionalities[vertex] in self.direction_arrows_list[1]:
                new_edge_direction = "↕"
            self.graph_edges_directionalities[vertex] = new_edge_direction
            self.network_units_list[vertex[0]][vertex[1]].add_unit_edge_directionality(new_edge_direction)

    # it finds the nearest vertex to vertex0 in the graph computing the manhattan distance and returns the vertex and the minimum distance
    def get_nearest_vertex(self, vertex0):  # vertex0 is a tuple (x, y)
        min_distance = np.inf
        min_vertex = vertex0
        for vertex in self.graph_vertices:
            if vertex != vertex0:
                # compute the manhattan distance
                distance = netalg.get_manhattan_distance(vertex0, vertex)
                if distance < min_distance:
                    min_distance = distance
                    min_vertex = vertex
        return min_vertex, min_distance

    def add_delete_obstacles(self, event = None):
        self.obstacles_on_grid = 1 - self.obstacles_on_grid
        for graph_row in self.network_units_list:
            for graph_unit in graph_row:
                if graph_unit.kind in [0, 1]:
                    graph_unit.change_unit_kind_color(1 - graph_unit.kind)
    
    def update_graph_vertices(self, vertex):
        self.add_edge_control = "first_vertex"
        if vertex in self.graph_vertices:
            vertex_number = self.graph_vertices.index(vertex) + 1
            self.graph_vertices.remove(vertex)
            self.network_units_list[vertex[0]][vertex[1]].change_unit_kind_color(0)
            self.remove_unit_images_directionality(vertex)
            edges_removed = []
            for vertices_pair in self.graph_edges.keys():
                if vertex in vertices_pair:
                    edges_removed.append(vertices_pair)
            edges_removed_string = ""
            for edge_vertices in edges_removed:
                edges_removed_string += f"edge {list(self.graph_edges.keys()).index(edge_vertices) + 1}: {edge_vertices}\n"
            self.write_information_box("Info", "blue", f"Vertex ({vertex[0]}, {vertex[1]}) (number: {vertex_number}) was deleted from the graph. "\
                                                        +f"\nAlso, {len(edges_removed)} edges were deleted from the graph (their vertices are written right below):\n" + edges_removed_string)
            for edge_vertices in edges_removed:
                removed_edge = self.graph_edges.pop(edge_vertices)
                for edge_unit in removed_edge:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(0)
                    self.remove_unit_images_directionality(edge_unit)
        else:
            self.graph_vertices.append(vertex)
            self.network_units_list[vertex[0]][vertex[1]].change_unit_kind_color(2)
            self.write_information_box("Info", "blue", f"Vertex [{vertex[0]}, {vertex[1]}] (graph unit: {self.network_units_list[vertex[0]][vertex[1]].number}) was added to the graph.")
        self.update_vertices_edges_info_choose_list()

    def update_graph_edges(self, vertex):
        if self.add_edge_control == "first_vertex" and self.network_units_list[vertex[0]][vertex[1]].kind == 2:
            self.add_edge_control = "second_vertex"
            self.add_edge_first_vertex = vertex
        elif self.add_edge_control == "second_vertex" and self.network_units_list[vertex[0]][vertex[1]].kind == 2 and self.add_edge_first_vertex != vertex:
            vertices_pair = tuple(netalg.sort_vertices_by_manhattan_distance([self.add_edge_first_vertex, vertex], (0, 0)))
            if vertices_pair in self.graph_edges.keys():  # if the edge already exists, delete it
                removed_edge = self.graph_edges.pop(vertices_pair)
                for edge_unit in removed_edge:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(0)
                    self.remove_unit_images_directionality(edge_unit)
                self.write_information_box("Info", "blue", f"The edge between the vertices {vertices_pair[0]} and {vertices_pair[1]} was deleted from the graph.")
            else:  # if the edge does not exist, create it
                self.add_edge_control = "first_vertex"
                old_edges_number = len(self.graph_edges)
                for random_edge in [0, 1]:
                    self.create_new_edge(self.add_edge_first_vertex, vertex, random_edge)
                if len(self.graph_edges) > old_edges_number:
                    for edge_unit in self.graph_edges[vertices_pair]:
                        self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
                    self.write_information_box("Info", "blue", f"A new edge was added to the graph between the vertices {vertices_pair[0]} and {vertices_pair[1]}.")
        self.update_vertices_edges_info_choose_list()

    def update_vertices_edges_info_choose_list(self):
        self.vertices_num_indicator.configure(text = len(self.graph_vertices))
        self.edges_num_indicator.configure(text = len(self.graph_edges))
        if self.vertex_edge_choice == "vertex":
            self.vertices_edges_choose.configure(values = list(range(1, len(self.graph_vertices) + 1)))
        elif self.vertex_edge_choice == "edge":
            self.vertices_edges_choose.configure(values = list(range(1, len(self.graph_edges) + 1)))
        self.edges_directionality_choose.configure(values = list(range(1, len(self.graph_edges) + 1)))
        self.edges_directionality_choose.set("")

    def remove_unit_images_directionality(self, graph_unit):
        if self.network_units_list[graph_unit[0]][graph_unit[1]].has_edge_directionality:
            self.graph_edges_directionalities.pop(graph_unit)
            self.network_units_list[graph_unit[0]][graph_unit[1]].remove_unit_edge_directionality()
        if self.network_units_list[graph_unit[0]][graph_unit[1]].has_time_weight_image:
            self.network_units_list[graph_unit[0]][graph_unit[1]].remove_unit_time_weight_image()
        if self.network_units_list[graph_unit[0]][graph_unit[1]].has_target_vertex_image:
            self.network_units_list[graph_unit[0]][graph_unit[1]].remove_unit_target_vertex_image()

    def highlight_vertices_edges(self, event = None):
        if self.vertex_edge_choice == "vertex" and event.widget == self.vertices_edges_choose:
            vertex = self.graph_vertices[int(self.vertices_edges_choose.get()) - 1]
            self.network_place.itemconfigure(self.network_units_list[vertex[0]][vertex[1]].unit, fill = "magenta")
            self.network_place.update()
            t.sleep(0.5)
            self.network_units_list[vertex[0]][vertex[1]].change_unit_kind_color(2)
        elif event.widget == self.edges_directionality_choose or self.vertex_edge_choice == "edge" and event.widget == self.vertices_edges_choose:
            self.edges_directionality_choose.set(int(event.widget.get()))
            self.edges_directionality_change_button.configure(text = self.graph_edges_directionalities[list(self.graph_edges.values())[int(event.widget.get()) - 1][0]])
            edge = list(self.graph_edges.values())[int(event.widget.get()) - 1]
            for edge_unit in edge:
                self.network_place.itemconfigure(self.network_units_list[edge_unit[0]][edge_unit[1]].unit, fill = "magenta")
            self.network_place.update()
            t.sleep(0.5)
            for edge_unit in edge:
                self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
    
    def load_time_weight_image(self, time_weight_name):
        self.chosen_time_weight_image = Image.open(os.getcwd() + "/images/time_weights_images/" + time_weight_name + ".png")
        self.chosen_time_weight_image = self.chosen_time_weight_image.resize((self.time_weights_images_canvas_width, self.time_weights_images_canvas_height), Image.LANCZOS)
        self.chosen_time_weight_image = ImageTk.PhotoImage(self.chosen_time_weight_image, master = self.root)
        self.time_weights_images_canvas.create_image(self.time_weights_images_canvas_width / 2 + 2, self.time_weights_images_canvas_height / 2 + 2, image = self.chosen_time_weight_image)
    
    def load_target_vertex_image(self, target_vertex_name):
        self.chosen_target_vertex_image = Image.open(os.getcwd() + "/images/target_vertices_images/" + target_vertex_name + ".png")
        self.chosen_target_vertex_image = self.chosen_target_vertex_image.resize((self.target_vertices_images_canvas_width, self.target_vertices_images_canvas_height), Image.LANCZOS)
        self.chosen_target_vertex_image = ImageTk.PhotoImage(self.chosen_target_vertex_image, master = self.root)
        self.target_vertices_images_canvas.create_image(self.target_vertices_images_canvas_width / 2 + 2, self.target_vertices_images_canvas_height / 2 + 2, image = self.chosen_target_vertex_image)

    def save_graph(self, event = None):
        if len(self.graph_vertices) != 0 and len(self.graph_vertices) != 0:
            ask_graph_name = sd.askstring(parent = self.root, title = 'Graph name', prompt = 'Give the name of the graph:')
            if ask_graph_name != None and ask_graph_name != "":
                overwrite_file_accept = False
                if (ask_graph_name + ".txt") in os.listdir(os.getcwd() + "/saved_graphs"):
                    overwrite_file_accept = ms.askyesno("Question", "There is already a file with this name. Do you want to overwrite it?")
                if (ask_graph_name + ".txt") not in os.listdir(os.getcwd() + "/saved_graphs") or overwrite_file_accept:
                    graph_file = open(os.getcwd() + "/saved_graphs/{}.txt".format(ask_graph_name), "w", encoding = "UTF-8")
                    graph_file.write("{},{}\n".format(self.grid_rows, self.grid_columns))
                    for vertex_number, vertex_unit in enumerate(self.graph_vertices):
                        vertex_unit = self.network_units_list[vertex_unit[0]][vertex_unit[1]]
                        vertex_unit_info = ""
                        vertex_unit_info += str(vertex_number) + "," + str(vertex_unit.kind) + "," + str(vertex_unit.row) + "," + str(vertex_unit.column) + ","
                        vertex_unit_info += str(vertex_unit.target_vertex_name)
                        graph_file.write(vertex_unit_info + "\n")
                    for edge_number, edge in enumerate(list(self.graph_edges.values())):
                        for edge_unit in edge:
                            edge_unit = self.network_units_list[edge_unit[0]][edge_unit[1]]
                            vertices_pair = list(self.graph_edges.keys())[edge_number]
                            edge_unit_info = ""
                            edge_unit_info += str(edge_number) + "," + str(edge_unit.kind) + "," + str(edge_unit.row) + "," + str(edge_unit.column) + ","
                            edge_unit_info += str(edge_unit.time_weight_name) + "," + str(self.time_weights_values[edge_unit.time_weight_name]) + "," + str(edge_unit.edge_directionality_text) + ","
                            edge_unit_info += str(self.graph_vertices.index(vertices_pair[0]))  + "," + str(self.graph_vertices.index(vertices_pair[1]))
                            graph_file.write(edge_unit_info + "\n")
                    graph_file.close()
                    ms.showinfo("Information", "File saved successfully!")
                    self.get_saved_graphs()
                else:
                    ms.showinfo("Information", "File was not saved!")
        else:
            ms.showwarning("Warning", "There is nothing to save!")

    def load_graph(self, event = None):
        graph_file = self.saved_graph_choose.get() + ".txt"
        load_graph_accept = ms.askyesno("Question", "Are you sure you want to load the file {}?".format(graph_file))
        if load_graph_accept:
            self.graph_info = []
            with open(os.getcwd() + f"/saved_graphs/{graph_file}", "r", encoding = "UTF-8") as file:
                for line in file:
                    self.graph_info.append(line[:-1].split(","))
            self.grid_rows = int(self.graph_info[0][0])
            self.grid_columns = int(self.graph_info[0][1])
            self.new_network_grid()
            self.graph_units_info = self.graph_info[1:]
            vertices_counter = 0
            for vertex_unit in self.graph_units_info:
                if int(vertex_unit[1]) == 3:
                    break
                vertices_counter += 1
                self.graph_vertices.append((int(vertex_unit[2]), int(vertex_unit[3])))
                self.network_units_list[int(vertex_unit[2])][int(vertex_unit[3])].change_unit_kind_color(2)
                if str(vertex_unit[4]) != "default":
                    self.network_units_list[int(vertex_unit[2])][int(vertex_unit[3])].add_unit_target_vertex_image(str(vertex_unit[4]))
            for edge_unit in self.graph_units_info[vertices_counter:]:
                self.network_units_list[int(edge_unit[2])][int(edge_unit[3])].change_unit_kind_color(3)
                vertices_pair = (self.graph_vertices[int(edge_unit[7])], self.graph_vertices[int(edge_unit[8])])
                if vertices_pair not in list(self.graph_edges.keys()):
                    self.graph_edges[vertices_pair] = []
                    self.graph_edges[vertices_pair].append((int(edge_unit[2]), int(edge_unit[3])))
                    self.graph_edges_directionalities[(int(edge_unit[2]), int(edge_unit[3]))] = str(edge_unit[6])
                    self.network_units_list[int(edge_unit[2])][int(edge_unit[3])].add_unit_edge_directionality(str(edge_unit[6]))
                else:
                    self.graph_edges[vertices_pair].append((int(edge_unit[2]), int(edge_unit[3])))
                if str(edge_unit[4]) != "default":
                    self.network_units_list[int(edge_unit[2])][int(edge_unit[3])].add_unit_time_weight_image(str(edge_unit[4]))
                    self.time_weights_values[edge_unit[4]] = int(edge_unit[5])
                if str(edge_unit[6]) != "default":
                    self.graph_edges_directionalities[(int(edge_unit[2]), int(edge_unit[3]))] = str(edge_unit[6])
                    self.network_units_list[int(edge_unit[2])][int(edge_unit[3])].add_unit_edge_directionality(str(edge_unit[6]))
            self.update_vertices_edges_info_choose_list()
    
    def get_saved_graphs(self, event = None):
        self.saved_graph_choose.configure(values = [file[:-4] for file in os.listdir(os.getcwd() + "/saved_graphs")])

    def check_graph_connectivity(self, event = None):
        if len(self.graph_edges) != 0:  # first check if the network has edges (in order to be considered a graph) and if yes continue
            adjacency_list = netalg.create_adjacency_list(self.graph_vertices, list(self.graph_edges.keys()), [self.convert_edge_directionality(edge_number) for edge_number in range(len(self.graph_edges))])
            # print the adjacency list
            adjacency_list_string = "\nThe adjacency list of the graph is:\n"
            for order, vertices in enumerate(list(adjacency_list.values())):
                adjacency_list_string += f"Vertex {order + 1}: {vertices}\n"
            print(adjacency_list_string)
            print("The adjacency matrix of the graph is:")
            print(netalg.create_adjacency_matrix(self.graph_vertices, list(self.graph_edges.keys()), [self.convert_edge_directionality(edge_number) for edge_number in range(len(self.graph_edges))]))
            # show in the information box if the graph is connected or not and if not show the vertices that are not reachable from the vertex referred first
            not_visited_vertices_total, is_graph_connected = netalg.check_graph_connectivity(adjacency_list)
            is_graph_connected_string = "The graph is strongly connected." if is_graph_connected else "The graph is not strongly connected."
            not_visited_vertices_total_string = ""
            if not is_graph_connected:
                not_visited_vertices_total_string = "\nBelow are written the vertices that are not reachable from the vertex referred first:\n"
                for order, vertices in enumerate(not_visited_vertices_total):
                    if vertices != []:
                        not_visited_vertices_total_string += f"Vertex {order + 1}: {vertices}\n"
            self.write_information_box("Info", "blue", is_graph_connected_string + not_visited_vertices_total_string)
            return not_visited_vertices_total

    def there_is_a_solution(self, event = None):  # check if there is a solution for the problem
        if len(self.target_vertices) == 3 and len(self.graph_edges) != 0:
            # find the vertices of the ambulance, the patient and the hospital
            ambulance_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("ambulance")])
            patient_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("patient")])
            hospital_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("hospital")])
            # check if the patient is reachable from the ambulance and the hospital is reachable from the patient
            not_visited_vertices_total = self.check_graph_connectivity()
            if patient_vertex+1 in not_visited_vertices_total[ambulance_vertex]:
                self.write_information_box("Warning", "red", "The shortest path problem has no solution because the patient is not reachable from the ambulance.")
            if hospital_vertex+1 in not_visited_vertices_total[patient_vertex]:
                self.write_information_box("Warning", "red", "The shortest path problem has no solution because the hospital is not reachable from the patient.")
            if patient_vertex+1 not in not_visited_vertices_total[ambulance_vertex] and hospital_vertex+1 not in not_visited_vertices_total[patient_vertex]:
                return True
        else:
            self.write_information_box("Warning", "red", f"You have to first create a graph with all the 3 target vertices, before you try to solve it!")
        return False

    def find_shortest_path_with_IP(self, event = None):  # find the shortest path using the IP model
        if self.there_is_a_solution():
            self.IP_solve_mode = True
            self.enter_solve_mode()
            # find the vertices of the ambulance, the patient and the hospital
            ambulance_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("ambulance")])
            patient_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("patient")])
            hospital_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("hospital")])
            # create the decision variables that will be used in the optimization problem, based on the edges and their directionality
            adjacency_list = netalg.create_adjacency_list(self.graph_vertices, list(self.graph_edges.keys()), [self.convert_edge_directionality(edge_number) for edge_number in range(len(self.graph_edges))])
            for i in range(len(adjacency_list)):
                for j in list(adjacency_list.values())[i]:
                    self.decision_variables.append((list(adjacency_list.keys())[i]-1, j-1))
            self.decision_variables_names_list = []
            for decision_variable in self.decision_variables:
                self.decision_variables_names_list.append(f"x_{decision_variable[0]+1}_{decision_variable[1]+1}")
            # create a list of the edges these decision variables represent at the right order
            self.decision_edges_list = []
            for edge_vertices in self.decision_variables:
                vertices_pair_key = tuple(netalg.sort_vertices_by_manhattan_distance([self.graph_vertices[edge_vertices[0]], self.graph_vertices[edge_vertices[1]]], (0, 0)))
                self.decision_edges_list.append(vertices_pair_key)
            # initialize the matrix A and the vectors b_ap and b_ph
            # the matrix A of the coefficients of the constraints (the same for both paths)
            self.A = np.zeros((len(self.graph_vertices), len(self.decision_variables)), dtype = int)
            # the vector b at the right side of the constraints for the path from the ambulance to the patient
            self.b_ap = np.zeros((self.A.shape[0], 1), dtype = int)
            # the vector b at the right side of the constraints for the path from the patient to the hospital
            self.b_ph = np.zeros((self.A.shape[0], 1), dtype = int)
            # calculate weights of all the edges based on the current optimization criterion
            self.costs = []
            for edge in self.decision_edges_list:  # go through all the graph edges
                if self.optimization_type == "time":  # compute time weights of edges
                    self.costs.append(sum([self.time_weights_values[self.network_units_list[x][y].time_weight_name] for (x, y) in self.graph_edges[edge]]) + 1)
                elif self.optimization_type == "length":  # compute length weights of edges
                    self.costs.append(netalg.get_manhattan_distance(edge[0], edge[1]))
            # create the matrix A
            for i in range(self.A.shape[0]):
                for j in range(self.A.shape[1]):
                    if (i, j) in self.decision_variables:
                        self.A[i][self.decision_variables.index((i, j))] = 1
                    if (j, i) in self.decision_variables:
                        self.A[i][self.decision_variables.index((j, i))] = -1
            # create the vectors b_ap and b_ph
            self.b_ap[ambulance_vertex][0] = 1
            self.b_ap[patient_vertex][0] = -1
            self.b_ph[patient_vertex][0] = 1
            self.b_ph[hospital_vertex][0] = -1
            # create the vector c
            self.c = np.array(self.costs)  # the vector c of the coefficients of the objective function
            # print the matrix A and the vectors b and c of the two integer programming problems
            print(f"\nDecision variables = {self.decision_variables_names_list}\n")
            print(f"A (common for the two problems) = \n{self.A}\n")
            print(f"b for ambulance → patient: \n{self.b_ap}\n")
            print(f"b for patient → hospital: \n{self.b_ph}\n")
            print(f"c (the costs of movement): \n{self.c}\n")
            # find the shortest path from the ambulance to the patient and from the patient to the hospital
            self.shortest_path_ambulance_patient = grasol.solve_IP_problem(self.A, self.b_ap, self.c, self.decision_variables_names_list, path_name = "path_1")  # shortest path from the ambulance to the patient
            self.shortest_path_patient_hospital = grasol.solve_IP_problem(self.A, self.b_ph, self.c, self.decision_variables_names_list, path_name = "path_2")  # shortest path from the patient to the hospital
            self.graph_state_indicator.configure(text = "Shortest paths\nfound...")
            # find the edges units that are part of the shortest path from the ambulance to the patient
            self.shortest_path_ambulance_patient_units = []
            for dec_var_number, decision_value in enumerate(self.shortest_path_ambulance_patient):
                if decision_value == 1.0:
                    self.shortest_path_ambulance_patient_units += self.graph_edges[self.decision_edges_list[dec_var_number]]
            # find the edges units that are part of the shortest path from the patient to the hospital
            self.shortest_path_patient_hospital_units = []
            for dec_var_number, decision_value in enumerate(self.shortest_path_patient_hospital):
                if decision_value == 1.0:
                    self.shortest_path_patient_hospital_units += self.graph_edges[self.decision_edges_list[dec_var_number]]
            # write the information about the shortest paths inside the information box
            # find the total relative cost of the shortest paths for the current optimization criterion
            total_relative_cost_ambulance_patient = round(np.dot(self.c.T, np.array(self.shortest_path_ambulance_patient)))
            total_relative_cost_patient_hospital = round(np.dot(self.c.T, np.array(self.shortest_path_patient_hospital)))
            # find the total absolute cost of the shortest paths for the current optimization criterion
            if self.optimization_type == "time":
                measure_units = "sec"
                total_absolute_cost_ambulance_patient = round(3.6 * self.grid_units_size / self.normal_velocity * total_relative_cost_ambulance_patient, 2)
                total_absolute_cost_patient_hospital = round(3.6 * self.grid_units_size / self.normal_velocity * total_relative_cost_patient_hospital, 2)
            elif self.optimization_type == "length":
                measure_units = "m"
                total_absolute_cost_ambulance_patient = round(self.grid_units_size * total_relative_cost_ambulance_patient, 2)
                total_absolute_cost_patient_hospital = round(self.grid_units_size * total_relative_cost_patient_hospital, 2)
            self.write_information_box("Solved", "green", f"Shortest paths found with {self.optimization_type} being the optimization criterion!"+\
                                        f"\n1) Shortest path: ambulance → patient"+\
                                        f"\n    - Path edges: {[list(self.graph_edges.keys()).index(self.decision_edges_list[k])+1 for k in range(len(self.shortest_path_ambulance_patient)) if self.shortest_path_ambulance_patient[k] == 1.0]}"+\
                                        f"\n    - Total {self.optimization_type} cost: {total_relative_cost_ambulance_patient} ({total_absolute_cost_ambulance_patient} {measure_units})"+\
                                        f"\n2) Shortest path: patient → hospital"+\
                                        f"\n    - Path edges: {[list(self.graph_edges.keys()).index(self.decision_edges_list[k])+1 for k in range(len(self.shortest_path_patient_hospital)) if self.shortest_path_patient_hospital[k] == 1.0]}"+\
                                        f"\n    - Total {self.optimization_type} cost: {total_relative_cost_patient_hospital} ({total_absolute_cost_patient_hospital} {measure_units})")
            self.create_area_for_model_solution_IP()

    def find_shortest_path_with_Astar(self, event = None):
        if self.there_is_a_solution():
            self.IP_solve_mode = False
            self.enter_solve_mode()
            # find the vertices of the ambulance, the patient and the hospital
            ambulance_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("ambulance")])
            patient_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("patient")])
            hospital_vertex = self.graph_vertices.index(list(self.target_vertices.keys())[list(self.target_vertices.values()).index("hospital")])
            # find the adjacency list of the graph
            adjacency_list = netalg.create_adjacency_list(self.graph_vertices, list(self.graph_edges.keys()), [self.convert_edge_directionality(edge_number) for edge_number in range(len(self.graph_edges))])
            adjacency_list_2 = {}
            for i in range(len(adjacency_list)):
                # create a dictionary of the adjacency list, where the keys are the vertices and the values are the vertices that are adjacent to the key vertex (not the vertices numbers, but the vertices themselves)
                adjacency_list_2[self.graph_vertices[list(adjacency_list.keys())[i] - 1]] = [self.graph_vertices[x-1] for x in list(adjacency_list.values())[i]]
            costs_list = {}  # list (lexicon in fact) of costs of all the edges
            for edge in list(self.graph_edges.keys()):  # go through all the graph edges
                if self.optimization_type == "time":  # compute time weights of edges
                    costs_list[edge] = sum([self.time_weights_values[self.network_units_list[x][y].time_weight_name] for (x, y) in self.graph_edges[edge]]) + 1
                elif self.optimization_type == "length":  # compute length weights of edges
                    costs_list[edge] = netalg.get_manhattan_distance(edge[0], edge[1])
            self.shortest_path_ambulance_patient = netalg.A_star(adjacency_list_2, costs_list, self.graph_vertices[ambulance_vertex], self.graph_vertices[patient_vertex])
            self.shortest_path_patient_hospital = netalg.A_star(adjacency_list_2, costs_list, self.graph_vertices[patient_vertex], self.graph_vertices[hospital_vertex])
            self.graph_state_indicator.configure(text = "Shortest paths\nfound...")
            # find the edges units that are part of the shortest path from the ambulance to the patient
            self.shortest_path_ambulance_patient_units = []
            edges_on_shortest_path_ambulance_patient = []
            total_relative_cost_ambulance_patient = 0
            for k in range(len(self.shortest_path_ambulance_patient) - 1):
                edge_vertices_pair = tuple(netalg.sort_vertices_by_manhattan_distance([self.shortest_path_ambulance_patient[k], self.shortest_path_ambulance_patient[k+1]], (0, 0)))
                self.shortest_path_ambulance_patient_units += self.graph_edges[edge_vertices_pair]
                total_relative_cost_ambulance_patient += costs_list[edge_vertices_pair]  # compute the total relative cost of the shortest path from the ambulance to the patient
                edges_on_shortest_path_ambulance_patient.append(list(self.graph_edges.keys()).index(edge_vertices_pair)+1)
            # find the edges units that are part of the shortest path from the patient to the hospital
            self.shortest_path_patient_hospital_units = []
            edges_on_shortest_path_patient_hospital = []
            total_relative_cost_patient_hospital = 0
            for k in range(len(self.shortest_path_patient_hospital) - 1):
                edge_vertices_pair = tuple(netalg.sort_vertices_by_manhattan_distance([self.shortest_path_patient_hospital[k], self.shortest_path_patient_hospital[k+1]], (0, 0)))
                self.shortest_path_patient_hospital_units += self.graph_edges[edge_vertices_pair]
                total_relative_cost_patient_hospital += costs_list[edge_vertices_pair]  # compute the total relative cost of the shortest path from the patient to the hospital
                edges_on_shortest_path_patient_hospital.append(list(self.graph_edges.keys()).index(edge_vertices_pair)+1)
            # find the total absolute cost of the shortest paths for the current optimization criterion
            if self.optimization_type == "time":
                measure_units = "sec"
                total_absolute_cost_ambulance_patient = round(3.6 * self.grid_units_size / self.normal_velocity * total_relative_cost_ambulance_patient, 2)
                total_absolute_cost_patient_hospital = round(3.6 * self.grid_units_size / self.normal_velocity * total_relative_cost_patient_hospital, 2)
            elif self.optimization_type == "length":
                measure_units = "m"
                total_absolute_cost_ambulance_patient = round(self.grid_units_size * total_relative_cost_ambulance_patient, 2)
                total_absolute_cost_patient_hospital = round(self.grid_units_size * total_relative_cost_patient_hospital, 2)
            self.write_information_box("Solved", "green", f"Shortest paths found with {self.optimization_type} being the optimization criterion!"+\
                            f"\n1) Shortest path: ambulance → patient"+\
                            f"\n    - Path edges: {edges_on_shortest_path_ambulance_patient}"+\
                            f"\n    - Total {self.optimization_type} cost: {total_relative_cost_ambulance_patient} ({total_absolute_cost_ambulance_patient} {measure_units})"+\
                            f"\n2) Shortest path: patient → hospital"+\
                            f"\n    - Path edges: {edges_on_shortest_path_patient_hospital}"+\
                            f"\n    - Total {self.optimization_type} cost: {total_relative_cost_patient_hospital} ({total_absolute_cost_patient_hospital} {measure_units})")

    def create_area_for_model_solution_IP(self, event = None):
        solve_mode_panel_width, solve_mode_panel_height = self.solve_mode_panel["width"], self.solve_mode_panel["height"]
        solve_mode_panel_rows = 15
        solve_mode_panel_font = 15
        solve_mode_panel_bg_color = "black"
        p1t_ord, p1m_ord, p2t_ord, p2m_ord, smsb_ord = 1, 4, 8, 11, 15
        p1t_x, p1m_x, p2t_x, p2m_x, smsb_x = 1/2, 1/2, 1/2, 1/2, 1/2
        path_1_mod_sol_text_rows, path_2_mod_sol_text_rows = 18, 18
        path_1_mod_sol_text_columns, path_2_mod_sol_text_columns = 45, 45
        menu_label(self.solve_mode_panel, "Path 1 model and solution:", f"Arial {solve_mode_panel_font} bold underline", "gold", solve_mode_panel_bg_color, p1t_x * solve_mode_panel_width, p1t_ord * solve_mode_panel_height / (solve_mode_panel_rows + 1) - 10)
        self.path_1_mod_sol_text_box = tk.Text(self.solve_mode_panel, font = 'Calibri 10 bold', width = path_1_mod_sol_text_columns, height = path_1_mod_sol_text_rows, wrap = "word")
        self.path_1_mod_sol_text_box.place(x = p1m_x * solve_mode_panel_width, y = p1m_ord * solve_mode_panel_height / (solve_mode_panel_rows + 1), anchor = "center")
        menu_label(self.solve_mode_panel, "Path 2 model and solution:", f"Arial {solve_mode_panel_font} bold underline", "gold", solve_mode_panel_bg_color, p2t_x * solve_mode_panel_width, p2t_ord * solve_mode_panel_height / (solve_mode_panel_rows + 1) - 10)
        self.path_2_mod_sol_text_box = tk.Text(self.solve_mode_panel, font = 'Calibri 10 bold', width = path_2_mod_sol_text_columns, height = path_2_mod_sol_text_rows, wrap = "word")
        self.path_2_mod_sol_text_box.place(x = p2m_x * solve_mode_panel_width, y = p2m_ord * solve_mode_panel_height / (solve_mode_panel_rows + 1), anchor = "center")
        self.show_mod_sol_button = menu_button(self.solve_mode_panel, self.show_mod_sol, f"Calibri {solve_mode_panel_font + 5} bold", "white", solve_mode_panel_bg_color, smsb_x * solve_mode_panel_width, smsb_ord * solve_mode_panel_height / (solve_mode_panel_rows + 1), self.change_parameters_actions).button
        self.show_problems_model_solution_IP("model")
    
    def show_problems_model_solution_IP(self, show_file, event = None):
        self.path_1_mod_sol_text_box.delete('1.0', tk.END)
        self.path_2_mod_sol_text_box.delete('1.0', tk.END)
        # read the file "problem_{show_file}_path_1.txt" and write its content inside the self.path_1_model_text_box
        with open(f"problem_{show_file}_path_1.txt", "r") as file:
            self.path_1_mod_sol_text_box.insert(tk.END, file.read())
        # read the file "problem_{show_file}_path_2.txt" and write its content inside the self.path_2_model_text_box
        with open(f"problem_{show_file}_path_2.txt", "r") as file:
            self.path_2_mod_sol_text_box.insert(tk.END, file.read())

    def apply_sensitivity_analysis(self, event = None):  # create a sensitivity report and show it inside the information box
        if self.entered_solve_mode:
            if self.sens_analysis_enabled:
                self.graph_state_indicator.configure(text = "Applied sensitivity\n analysis...")
                # find the total cost of the current graph
                self.total_graph_cost = sum(self.c)
                # ranges_till are the upper bounds for the cost the edges that are part of the shortest path and
                # ranges from are the lower bounds for the cost the edges that are not part of the shortest path
                self.ranges_till_path_1, self.ranges_from_path_1, self.ranges_till_path_2, self.ranges_from_path_2 = "", "", "", ""
                self.ranges_till_paths = [self.ranges_till_path_1, self.ranges_till_path_2]
                self.ranges_from_paths = [self.ranges_from_path_1, self.ranges_from_path_2]
                self.old_shortest_paths = [self.shortest_path_ambulance_patient, self.shortest_path_patient_hospital]
                self.b_vectors = [self.b_ap, self.b_ph]
                # apply sensitivity analysis on both shortest paths (path_1 and path_2)
                for path_order in range(2):  # path_order = 0 for path_1 and path_order = 1 for path_2
                    for cost_order in range(len(self.c)):  # cost_order = 0 for the first cost, cost_order = 1 for the second cost, etc.
                        if self.old_shortest_paths[path_order][cost_order] == 1:  # the edge is part of the shortest path
                            # initialize the left index to be the cost of the edge (the minimum time cost) and the right index to be the total cost of the graph (the maximum time cost)
                            left_index = self.c[cost_order]  # left index of the binary search
                            right_index = self.total_graph_cost  # right index of the binary search
                            costs_check = np.copy(self.c)  # the costs_check are set to be the same as the original
                            # we want to check if the path's cost is changed at the extreme cost case, if not then the edge cost does not affect the shortest path
                            costs_check = self.change_cost(costs_check, right_index, cost_order)
                            if np.dot(costs_check, self.old_shortest_paths[path_order]) !=\
                                np.dot(costs_check, grasol.solve_IP_problem(self.A, self.b_vectors[path_order], costs_check, self.decision_variables_names_list)):
                                costs_sens = np.copy(self.c)  # the costs_sens of the edges are initialized to be the same as the original
                                while left_index <= right_index:  # implementing binary search
                                    middle_index = (left_index + right_index) // 2  # middle index of the binary search
                                    new_costs = self.change_cost(costs_sens, middle_index, cost_order)  # attempt to change the cost of the current edge
                                    new_shortest_path = grasol.solve_IP_problem(self.A, self.b_vectors[path_order], new_costs, self.decision_variables_names_list)  # find the new shortest path
                                    # when a new shortest path is found (with lower total cost than the previous shortest path) while changing the cost of the current edge do the following
                                    if np.dot(new_costs, self.old_shortest_paths[path_order]) == np.dot(new_costs, new_shortest_path):
                                        costs_sens = self.change_cost(costs_sens, middle_index, cost_order)  # confirm the change of the cost of the current edge
                                        left_index = middle_index + 1
                                    else:
                                        right_index = middle_index - 1
                                if costs_sens[cost_order] != self.total_graph_cost:
                                    # the edge cost affects the shortest path
                                    cost_difference = costs_sens[cost_order] - self.c[cost_order]
                                    self.ranges_till_paths[path_order] += f"{list(self.graph_edges.keys()).index(self.decision_edges_list[cost_order]) + 1}"+\
                                                                            f" → {costs_sens[cost_order]} (+{cost_difference}), "
                        else:  # the edge is not part of the shortest path
                            # initialize the left index to be the length cost of the edge (the minimum time cost) and the right index to be the time cost of the edge (the maximum time cost)
                            left_index = netalg.get_manhattan_distance(self.decision_edges_list[cost_order][0], self.decision_edges_list[cost_order][1])  # left index of the binary search
                            right_index = self.c[cost_order]  # right index of the binary search
                            costs_check = np.copy(self.c)  # the costs_check are set to be the same as the original
                            # we want to check if the path's cost is changed at the extreme cost case, if not then the edge cost does not affect the shortest path
                            costs_check = self.change_cost(costs_check, left_index, cost_order)
                            if np.dot(costs_check, self.old_shortest_paths[path_order]) !=\
                                np.dot(costs_check, grasol.solve_IP_problem(self.A, self.b_vectors[path_order], costs_check, self.decision_variables_names_list)):
                                costs_sens = np.copy(self.c)  # the costs_sens of the edges are initialized to be the same as the original
                                while left_index <= right_index:  # implementing binary search
                                    middle_index = (left_index + right_index) // 2  # middle index of the binary search
                                    new_costs = self.change_cost(costs_sens, middle_index, cost_order)  # attempt to change the cost of the current edge
                                    new_shortest_path = grasol.solve_IP_problem(self.A, self.b_vectors[path_order], new_costs, self.decision_variables_names_list)  # find the new shortest path
                                    # when a new shortest path is found (with lower total cost than the previous shortest path) while changing the cost of the current edge do the following
                                    if np.dot(new_costs, self.old_shortest_paths[path_order]) == np.dot(new_costs, new_shortest_path):
                                        costs_sens = self.change_cost(costs_sens, middle_index, cost_order)  # confirm the change of the cost of the current edge
                                        right_index = middle_index - 1
                                    else:
                                        left_index = middle_index + 1
                                if costs_sens[cost_order] != netalg.get_manhattan_distance(self.decision_edges_list[cost_order][0], self.decision_edges_list[cost_order][1]):
                                    # the edge cost affects the shortest path
                                    cost_difference = self.c[cost_order] - costs_sens[cost_order]
                                    self.ranges_from_paths[path_order] += f"{list(self.graph_edges.keys()).index(self.decision_edges_list[cost_order]) + 1}"+\
                                                                            f" → {costs_sens[cost_order]} (-{cost_difference}), "
                self.write_information_box("Sens.", "brown", "Sensitivity analysis applied succesfully! Below are written the upper bounds for the costs of 'Edges on' (edges on the current shortest path) "+\
                                            "and the lower bounds for the costs of 'Edges off' (edges off the current shortest path), such that the shortest paths remain the same. "+\
                                            "If you increase any cost of the 'Edges on' or decrease any cost of the 'Edges off' by just 1, the shortest paths are going to change "+\
                                            "(and their total costs will be less than the total costs of the previous shortest paths)."
                                            f"\n1) For shortest path 1:"+\
                                            f"\n    - Edges on: {self.ranges_till_paths[0][:-2]}"+\
                                            f"\n    - Edges off: {self.ranges_from_paths[0][:-2]}"+\
                                            f"\n2) For shortest path 2:"+\
                                            f"\n    - Edges on: {self.ranges_till_paths[1][:-2]}"+\
                                            f"\n    - Edges off: {self.ranges_from_paths[1][:-2]}")
            else:
                self.write_information_box("Warning", "red", f"Sensitivity analysis is only applicable when solving the problem with the IP solver and also time needs to be the optimization criterion!")
        else:
            self.write_information_box("Warning", "red", f"You have to first enter solve mode, before you try to apply sensitivity analysis!")
    
    def change_cost(self, old_costs, new_cost, new_cost_index, event = None):  # changes the cost at the given index of the old_costs list and returns the new_costs list
        new_costs = np.copy(old_costs)
        new_costs[new_cost_index] = new_cost
        return new_costs

    def show_path_1(self, event = None):
        if self.entered_solve_mode:
            if self.showing_path_1:
                # unhighlight the units that are part of the shortest path from the ambulance to the patient
                self.showing_path_1 = False
                for edge_unit in self.shortest_path_ambulance_patient_units:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
            else:
                # highlight the units that are part of the shortest path from the ambulance to the patient
                self.showing_path_1 = True
                for edge_unit in self.shortest_path_ambulance_patient_units:
                    self.network_place.itemconfigure(self.network_units_list[edge_unit[0]][edge_unit[1]].unit, fill = self.path_1_color)
                    self.network_units_list[edge_unit[0]][edge_unit[1]].color = self.path_1_color
                self.network_place.update()
    
    def show_path_2(self, event = None):
        if self.entered_solve_mode:
            if self.showing_path_2:
                # unhighlight the units that are part of the shortest path from the patient to the hospital
                self.showing_path_2 = False
                for edge_unit in self.shortest_path_patient_hospital_units:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
            else:
                # unhighlight the units that are part of the shortest path from the patient to the hospital
                self.showing_path_2 = True
                for edge_unit in self.shortest_path_patient_hospital_units:
                    self.network_place.itemconfigure(self.network_units_list[edge_unit[0]][edge_unit[1]].unit, fill = self.path_2_color)
                    self.network_units_list[edge_unit[0]][edge_unit[1]].color = self.path_2_color
                self.network_place.update()

    def enter_solve_mode(self, event = None):
        if self.entered_solve_mode:
            self.exit_solve_mode()
            self.enter_solve_mode()
        else:
            self.entered_solve_mode = True
            self.shortest_path_ambulance_patient_units = []
            self.shortest_path_patient_hospital_units = []
            self.showing_path_1 = False
            self.showing_path_2 = False
            self.decision_variables = []
            if self.optimization_type == "time" and self.IP_solve_mode:
                self.sens_analysis_enabled = True
            self.IP_solve_mode = False
            self.add_delete_obstacles()
            self.graph_state_indicator.configure(text = "Searching for the\nshortest path...")
            self.solve_mode_panel = tk.Frame(self.menus_background_1, width = 1/2 * self.menus_background_2["width"], height = self.menus_background_2["height"], bg = "black", highlightbackground = "black", highlightthickness = 5)
            self.solve_mode_panel.grid(row = 0, column = 0, rowspan = 2, sticky = tk.NSEW)
            self.network_place.update()

    def exit_solve_mode(self, event = None):
        if self.entered_solve_mode:
            self.entered_solve_mode = False
            self.sens_analysis_enabled = False
            self.solve_mode_panel.destroy()
            self.add_delete_obstacles()
            for edge_unit in self.shortest_path_ambulance_patient_units:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
            for edge_unit in self.shortest_path_patient_hospital_units:
                    self.network_units_list[edge_unit[0]][edge_unit[1]].change_unit_kind_color(3)
            self.graph_state_indicator.configure(text = "Just exited\nsolve mode...")
            self.network_place.update()

    def change_parameters_actions(self, event):  # change the network parameters and make actions
        if event.widget == self.grid_gap_button:  # change the grid_gap (the gap between the network units)
            self.grid_gap = self.alternate_matrix_elements(self.grid_gaps_list, self.grid_gap)
            self.grid_gap_button.configure(text = self.grid_gap)
            self.same_network_grid()
        elif event.widget == self.grid_color_theme_button:  # change the network units color theme
            self.grid_color_theme = 1 - self.grid_color_theme
            self.grid_color_theme_button.configure(text = self.grid_color_themes_list[self.grid_color_theme])
            self.time_weights_images_canvas.configure(bg = [self.units_colors[0][3], self.units_colors[1][3]][self.grid_color_theme])
            self.target_vertices_images_canvas.configure(bg = [self.units_colors[0][2], self.units_colors[1][2]][self.grid_color_theme])
            self.same_network_grid()
        elif event.widget == self.grid_units_size_button:  # change the network units size
            self.grid_units_size = self.alternate_matrix_elements(self.grid_units_sizes_list, self.grid_units_size)
            self.grid_units_size_button.configure(text = self.grid_units_size)
            self.grid_length_indicator.configure(text = self.grid_units_size * self.grid_rows)
            self.grid_width_indicator.configure(text = self.grid_units_size * self.grid_columns)
            self.normal_time_cost_indicator.configure(text = round(3.6 * self.grid_units_size / self.normal_velocity, 3))
        elif event.widget == self.normal_velocity_button:  # change the normal_velocity
            self.normal_velocity = self.alternate_matrix_elements(self.normal_velocities_list, self.normal_velocity)
            self.normal_velocity_button.configure(text = self.normal_velocity)
            self.normal_time_cost_indicator.configure(text = round(3.6 * self.grid_units_size / self.normal_velocity, 3))
        if event.widget.winfo_parent() == str(self.grid_menu):  # change the grid parameters
            if event.widget == self.grid_rows_button:  # change the grid_rows
                self.grid_rows = self.alternate_matrix_elements(self.grid_rows_list, self.grid_rows)
                self.grid_rows_button.configure(text = self.grid_rows)
            elif event.widget == self.grid_columns_button:  # change the grid_columns
                self.grid_columns = self.alternate_matrix_elements(self.grid_columns_list, self.grid_columns)
                self.grid_columns_button.configure(text = self.grid_columns)
        elif event.widget.winfo_parent() == str(self.create_graph_menu):
            if event.widget == self.auto_vertices_num_button:  # change the auto_vertices_number
                self.auto_vertices_num = self.alternate_matrix_elements(self.auto_vertices_num_list, self.auto_vertices_num)
                self.auto_vertices_num_button.configure(text = self.auto_vertices_num)
            elif event.widget == self.random_vertices_edges_choose_button:  # change the random_vertices_edges mode
                self.random_vertices_edges_choose = self.alternate_matrix_elements(self.random_vertices_edges_choose_list, self.random_vertices_edges_choose)
                self.random_vertices_edges_choose_button.configure(text = self.random_vertices_edges_choose)
            elif event.widget == self.create_graph_manually_button:  # change the create_graph_manually_mode
                self.create_graph_manually_mode = self.alternate_matrix_elements(self.create_graph_manually_modes_list, self.create_graph_manually_mode)
                self.create_graph_manually_button.configure(text = self.create_graph_manually_mode)
                if self.create_graph_manually_mode == "start":
                    self.graph_state_indicator.configure(text = "")
                    self.write_information_box("Info", "blue", "Creating graphs manually mode is disabled.")
                elif self.create_graph_manually_mode == "stop":
                    self.highlight_vertices_edges_mode = "start"
                    self.highlight_vertices_edges_button.configure(text = self.highlight_vertices_edges_mode)
                    self.graph_state_indicator.configure(text = "Creating\ngraphs manually ...")
                    self.write_information_box("Guide", "brown", "You can now create your own graph adding/deleting vertices (left-clicking) and edges (successively right-clicking the two vertices of the edge).")
            elif event.widget == self.time_weights_next_image_button:  # change the time_weights_image based on the current time weight name
                self.time_weight_name_current = self.alternate_matrix_elements(self.time_weights_names_list, self.time_weight_name_current)
                self.load_time_weight_image(self.time_weight_name_current)
                self.time_weights_values_button.configure(text = self.time_weights_values[self.time_weight_name_current])
                self.write_information_box("Info", "blue", f"Time weight: {self.time_weight_name_current}"\
                                                            +f"\nRelative time cost: {self.time_weights_values[self.time_weight_name_current]}"\
                                                            +f"\nAbsolute time cost: {round(3.6 * self.grid_units_size / self.normal_velocity * self.time_weights_values[self.time_weight_name_current], 2)} sec")
            elif event.widget == self.add_time_weights_button:
                self.add_time_weights_mode = self.alternate_matrix_elements(self.add_time_weights_modes_list, self.add_time_weights_mode)
                self.add_time_weights_button.configure(text = self.add_time_weights_mode)
                if self.add_time_weights_mode == "start":
                    self.graph_state_indicator.configure(text = "")
                    self.write_information_box("Info", "blue", "Adding time weights mode is disabled.")
                elif self.add_time_weights_mode == "stop":
                    self.graph_state_indicator.configure(text = "Adding\ntime weights ...")
                    self.write_information_box("Guide", "brown", f"You can now add time weights at an edge unit of the graph by middle-clicking on it. If you want to remove the time weight, middle-click on the corresponding unit again.")
            elif event.widget == self.time_weights_values_button:  # change the time_weights_values for the current time weight name
                self.time_weights_values[self.time_weight_name_current] = self.alternate_matrix_elements(self.time_weights_values_list, self.time_weights_values[self.time_weight_name_current])
                self.time_weights_values_button.configure(text = self.time_weights_values[self.time_weight_name_current])
            elif event.widget == self.edges_directionality_change_button:  # change the directionality of the current edge
                choose_directions_list = [True, False].index(self.edges_directionality_change_button["text"] in self.direction_arrows_list[0])
                new_edge_direction = self.alternate_matrix_elements(self.direction_arrows_list[choose_directions_list], self.edges_directionality_change_button["text"])
                self.edges_directionality_change_button.configure(text = new_edge_direction)
                if self.graph_edges_directionalities != {} and self.edges_directionality_choose.get() != "":
                    first_edge_unit = list(self.graph_edges.values())[int(self.edges_directionality_choose.get()) - 1][0]
                    self.graph_edges_directionalities[first_edge_unit] = new_edge_direction
                    self.network_units_list[first_edge_unit[0]][first_edge_unit[1]].add_unit_edge_directionality(new_edge_direction)
        elif event.widget.winfo_parent() == str(self.graph_info_menu):
            if event.widget == self.vertex_edge_choice_button:  # change the vertex_edge_choice
                self.vertex_edge_choice = self.alternate_matrix_elements(self.vertex_edge_choices_list, self.vertex_edge_choice)
                self.update_vertices_edges_info_choose_list()
                self.vertex_edge_choice_button.configure(text = self.vertex_edge_choice)
            elif event.widget == self.highlight_vertices_edges_button:  # change the highlight_vertices_edges_mode
                self.highlight_vertices_edges_mode = self.alternate_matrix_elements(self.highlight_vertices_graph_edges_modes_list, self.highlight_vertices_edges_mode)
                self.highlight_vertices_edges_button.configure(text = self.highlight_vertices_edges_mode)
                if self.highlight_vertices_edges_mode == "start":
                    self.graph_state_indicator.configure(text = "")
                    self.write_information_box("Info", "blue", "Highlighting graph units mode is disabled.")
                elif self.highlight_vertices_edges_mode == "stop":
                    self.create_graph_manually_mode = "start"
                    self.create_graph_manually_button.configure(text = self.create_graph_manually_mode)
                    self.graph_state_indicator.configure(text = "Highlighting\ngraph units ...")
                    self.write_information_box("Guide", "brown", f"You can now select (left-clicking) vertices and edges on the graph and receive useful information about them.")
        elif event.widget.winfo_parent() == str(self.solve_problem_menu):  # change the optimization problem parameters
            if event.widget == self.target_vertices_next_image_button:  # change the target_vertex_image based on the current target vertex name
                self.target_vertex_name_current = self.alternate_matrix_elements(self.target_vertices_names_list, self.target_vertex_name_current)
                self.load_target_vertex_image(self.target_vertex_name_current)
                self.write_information_box("Info", "blue", f"Target vertex: {self.target_vertex_name_current}")
            elif event.widget == self.add_target_vertices_button:
                self.add_target_vertices_mode = self.alternate_matrix_elements(self.add_target_vertices_modes_list, self.add_target_vertices_mode)
                self.add_target_vertices_button.configure(text = self.add_target_vertices_mode)
                if self.add_target_vertices_mode == "start":
                    self.graph_state_indicator.configure(text = "")
                    self.write_information_box("Info", "blue", "Adding target vertices mode is disabled.")
                elif self.add_target_vertices_mode == "stop":
                    self.graph_state_indicator.configure(text = "Adding\ntarget vertices ...")
                    self.write_information_box("Guide", "brown", f"You can now add target vertices at a vertex unit of the graph by middle-clicking on it. If you want to remove the target vertex, middle-click on the corresponding unit again.")
            if event.widget == self.opt_problem_choose_button:  # change the optimization problem type
                self.optimization_type = self.alternate_matrix_elements(self.optimization_types_list, self.optimization_type)
                self.opt_problem_choose_button.configure(text = self.optimization_type)
        if self.entered_solve_mode:
            # change the show_mod_sol based on what must be shown (the model or the solution)
            if event.widget.winfo_parent() == str(self.solve_mode_panel) and event.widget == self.show_mod_sol_button:
                self.show_mod_sol = self.alternate_matrix_elements(self.show_mod_sol_list, self.show_mod_sol)
                self.show_mod_sol_button.configure(text = self.show_mod_sol)
                self.show_problems_model_solution_IP(self.show_mod_sol)
    
    def write_information_box(self, note, note_color, message):
        self.text_pointer = self.information_box.index('end')
        self.information_box.insert('end', "\n----------------------------------------------------------------------\n{}: {}".format(note, message))
        self.information_box.tag_add('{}'.format(note), str(float(self.text_pointer) + 1.0), format(float(self.text_pointer) + 1.00 + float(len(note) / 100), ".2f"))
        self.information_box.tag_configure('{}'.format(note), foreground = note_color, font = 'Arial 10 bold italic')
        self.information_box.see('end')

    def alternate_matrix_elements(self, matrix, index_element):  # alternate the game parametres that are inside the matrix based on the current index_element
        return (matrix[1:] + [matrix[0]])[matrix.index(index_element)]


class network_unit():
    def __init__(self, instance, grid_background, kind, number, row, column, width, height, border, color):
        self.instance = instance
        self.grid_background = grid_background
        self.kind = kind  # 0: none, 1: obstacle, 2: vertex, 3: edge
        self.number = number
        self.row = row
        self.column = column
        self.width = width
        self.height = height
        self.border = border
        self.color = color
        self.time_weight_name = "default"
        self.has_time_weight_image = False
        self.time_weight_image = None
        self.target_vertex_name = "default"
        self.has_target_vertex_image = False
        self.target_vertex_image = None
        self.edge_directionality_text = "default"
        self.has_edge_directionality = False
        self.canvas_offset = 2
        self.set_unit_on_grid()
    
    def set_unit_on_grid(self):
        try:
            self.grid_background.delete(self.unit)
        except AttributeError:
            pass
        self.unit = self.grid_background.create_rectangle([self.column * self.width + self.canvas_offset, self.row * self.height + self.canvas_offset, \
                                                        (self.column + 1) * self.width + self.canvas_offset, (self.row + 1) * self.height + self.canvas_offset], \
                                                        fill = self.color, width = self.border, outline = "orange", tags = f"unit_{self.number}_{self.row}_{self.column}")
        self.grid_background.tag_bind(f"unit_{self.number}_{self.row}_{self.column}", "<Button-1>", self.left_click_unit)
        self.grid_background.tag_bind(f"unit_{self.number}_{self.row}_{self.column}", "<Button-3>", self.right_click_unit)
        self.grid_background.tag_bind(f"unit_{self.number}_{self.row}_{self.column}", "<Button-2>", self.middle_click_unit)
        self.grid_background.tag_bind(f"unit_{self.number}_{self.row}_{self.column}", "<Enter>", self.highlight_unit)
        self.grid_background.tag_bind(f"unit_{self.number}_{self.row}_{self.column}", "<Leave>", self.unhighlight_unit)
    
    def change_unit_kind_color(self, unit_kind):
        self.kind = unit_kind
        self.color = networks_list[self.instance].units_colors[networks_list[self.instance].grid_color_theme][self.kind]
        self.grid_background.itemconfigure(self.unit, fill = self.color)
    
    def add_unit_time_weight_image(self, new_time_weight_name):
        self.has_time_weight_image = True
        self.time_weight_name = new_time_weight_name
        self.time_weight_image = Image.open(os.getcwd() + "/images/time_weights_images/" + self.time_weight_name + ".png")
        self.time_weight_image = self.time_weight_image.resize((int(self.width / 1.4), int(self.height / 1.4)), Image.LANCZOS)
        self.time_weight_image = ImageTk.PhotoImage(self.time_weight_image, master = networks_list[self.instance].root)
        self.grid_background.create_image((self.column + 1/2)* self.width + self.canvas_offset, (self.row + 1/2) * self.height + self.canvas_offset, image = self.time_weight_image)
        networks_list[self.instance].write_information_box("Info", "blue", f"Time weight \"{self.time_weight_name}\" with value {networks_list[self.instance].time_weights_values[self.time_weight_name]} was added to the edge unit ({self.row}, {self.column}).")
    
    def add_unit_target_vertex_image(self, new_target_vertex_name):
        if new_target_vertex_name not in list(networks_list[self.instance].target_vertices.values()):
            self.has_target_vertex_image = True
            self.target_vertex_name = new_target_vertex_name
            networks_list[self.instance].target_vertices[(self.row, self.column)] = self.target_vertex_name
            self.target_vertex_image = Image.open(os.getcwd() + "/images/target_vertices_images/" + self.target_vertex_name + ".png")
            self.target_vertex_image = self.target_vertex_image.resize((int(self.width / 1.4), int(self.height / 1.4)), Image.LANCZOS)
            self.target_vertex_image = ImageTk.PhotoImage(self.target_vertex_image, master = networks_list[self.instance].root)
            self.grid_background.create_image((self.column + 1/2)* self.width + self.canvas_offset, (self.row + 1/2) * self.height + self.canvas_offset, image = self.target_vertex_image)
            networks_list[self.instance].write_information_box("Info", "blue", f"Target vertex \"{self.target_vertex_name}\" was added to the vertex unit ({self.row}, {self.column}).")
        else:
            networks_list[self.instance].write_information_box("Warning", "red", f"You are allowed to place the target vertex \"{new_target_vertex_name}\" only on one vertex! This rule applies to all target vertices!")

    def add_unit_edge_directionality(self, new_edge_direction):
        self.set_unit_on_grid()
        self.has_edge_directionality = True
        self.edge_directionality_text = new_edge_direction
        self.grid_background.create_text((self.column + 1/2)* self.width + self.canvas_offset, (self.row + 1/2) * self.height + self.canvas_offset,\
                                         text = self.edge_directionality_text, font = f"Arial {int(min(self.width, self.height)/2)} bold", fill = "black")
        if self.has_time_weight_image:
            self.add_unit_time_weight_image(self.time_weight_name)
    
    def remove_unit_time_weight_image(self):
        self.set_unit_on_grid()
        if self.has_edge_directionality:
            self.add_unit_edge_directionality(networks_list[self.instance].graph_edges_directionalities[(self.row, self.column)])
        networks_list[self.instance].write_information_box("Info", "blue", f"Time weight \"{self.time_weight_name}\" with value {networks_list[self.instance].time_weights_values[self.time_weight_name]} was removed from the edge unit ({self.row}, {self.column}).")
        self.has_time_weight_image = False
        self.time_weight_name = "default"
        self.time_weight_image = None
    
    def remove_unit_target_vertex_image(self):
        networks_list[self.instance].target_vertices.pop((self.row, self.column))
        self.has_target_vertex_image = False
        self.target_vertex_name = "default"
        self.target_vertex_image = None
        self.set_unit_on_grid()
        networks_list[self.instance].write_information_box("Info", "blue", f"Target vertex \"{self.target_vertex_name}\" was removed from the vertex unit ({self.row}, {self.column}).")
    
    def remove_unit_edge_directionality(self):
        self.has_edge_directionality = False
        self.edge_directionality_text = "default"
        self.set_unit_on_grid()
    
    def highlight_unit(self, event):
        self.grid_background.itemconfigure(self.unit, fill = "magenta")
    
    def unhighlight_unit(self, event):
        self.grid_background.itemconfigure(self.unit, fill = self.color)
    
    def left_click_unit(self, event = None):
        if networks_list[self.instance].create_graph_manually_mode == "stop" and not networks_list[self.instance].entered_solve_mode:  # if the create_graph_manually_mode is enabled
            if self.kind == 0:  # if the unit is "none" change it to "vertex" (add a vertex on the graph)
                nearest_vertex, min_distance = networks_list[self.instance].get_nearest_vertex((self.row, self.column))
                if min_distance == 1:
                    networks_list[self.instance].write_information_box("Warning", "red", f"It's not allowed to place a vertex right next to the vertex {nearest_vertex}! All the vertices must have manhattan distance of at least 2!")
                else:
                    networks_list[self.instance].update_graph_vertices((self.row, self.column))
            elif self.kind == 2:  # if the unit is "vertex" change it to "none" (delete the vertex from the graph)
                networks_list[self.instance].update_graph_vertices((self.row, self.column))
            elif self.kind == 1:
                networks_list[self.instance].write_information_box("Guide", "brown", "You have to first delete the obstacles before you try to add a vertex on the graph.")
            elif self.kind == 3:
                networks_list[self.instance].write_information_box("Warning", "red", "It's not allowed to place a vertex on an edge! First delete the edge and then add the vertex!")
        elif networks_list[self.instance].highlight_vertices_edges_mode == "stop":  # if the highlight_vertices_edges_mode is enabled
            if self.kind == 2:  # give information to the user about the corresponding vertex
                vertex_number = networks_list[self.instance].graph_vertices.index((self.row, self.column)) + 1
                if self.has_target_vertex_image: name = self.target_vertex_name
                else: name = "None"
                networks_list[self.instance].write_information_box("Info", "blue", f"This is a graph vertex."\
                                                                                    +f"\nNumber: {vertex_number}"\
                                                                                    +f"\nCoordinates: ({self.row}, {self.column})"\
                                                                                    +f"\nTarget contained: {name}")
            elif self.kind == 3:  # give information to the user about the corresponding edge
                for edge_number, edge in enumerate(list(networks_list[self.instance].graph_edges.values())):
                    if (self.row, self.column) in edge:
                        networks_list[self.instance].edges_directionality_choose.set(edge_number + 1)
                        networks_list[self.instance].edges_directionality_change_button.configure(text = networks_list[self.instance].graph_edges_directionalities[edge[0]])
                        edge_vertices = list(networks_list[self.instance].graph_edges.keys())[edge_number]
                        vertex_1_number = networks_list[self.instance].graph_vertices.index(edge_vertices[0]) + 1
                        vertex_2_number = networks_list[self.instance].graph_vertices.index(edge_vertices[1]) + 1
                        relative_time_cost = sum([networks_list[self.instance].time_weights_values[networks_list[self.instance].network_units_list[x][y].time_weight_name] for (x, y) in edge]) + 1
                        absolute_time_cost = round(3.6 * networks_list[self.instance].grid_units_size / networks_list[self.instance].normal_velocity * relative_time_cost, 2)
                        networks_list[self.instance].write_information_box("Info", "blue", f"This is a graph edge."\
                                                                                            +f"\nNumber: {edge_number + 1}"\
                                                                                            +f"\nVertices and directionality: {vertex_1_number}:{edge_vertices[0]} {networks_list[self.instance].convert_edge_directionality(edge_number)} {vertex_2_number}:{edge_vertices[1]}"\
                                                                                            +f"\nRelative length cost: {len(edge) + 1}"\
                                                                                            +f"\nAbsolute length cost: {networks_list[self.instance].grid_units_size * (len(edge) + 1)} m"\
                                                                                            +f"\nRelative time cost: {relative_time_cost}"\
                                                                                            +f"\nAbsolute time cost: {absolute_time_cost} sec")
                        break
            elif self.kind == 0:
                networks_list[self.instance].write_information_box("Info", "blue", f"This is an empty graph unit.")
            elif self.kind == 1:
                networks_list[self.instance].write_information_box("Info", "blue", f"This is a graph obstacle.")
    
    def right_click_unit(self, event = None):
        if networks_list[self.instance].create_graph_manually_mode == "stop" and self.kind == 2 and not networks_list[self.instance].entered_solve_mode:  # if the create_graph_manually_mode is enabled and right-clicking on a vertex
            networks_list[self.instance].update_graph_edges((self.row, self.column))
    
    def middle_click_unit(self, event = None):
        if not networks_list[self.instance].entered_solve_mode:
            if networks_list[self.instance].add_time_weights_mode == "stop" and self.kind == 3:  # if the add_time_weights_mode is enabled and middle-clicking on an edge unit
                if self.time_weight_name == "default":  # if the unit doesn't have a time weight image, add it
                    self.add_unit_time_weight_image(networks_list[self.instance].time_weight_name_current)
                elif self.time_weight_name != "default":  # if the unit has a time weight image, remove it
                    self.remove_unit_time_weight_image()
            elif networks_list[self.instance].add_target_vertices_mode == "stop" and self.kind == 2:  # if the add_target_vertices_mode is enabled and middle-clicking on a vertex unit
                if self.target_vertex_name == "default":  # if the unit doesn't have a target vertex image, add it
                    self.add_unit_target_vertex_image(networks_list[self.instance].target_vertex_name_current)
                elif self.target_vertex_name != "default":  # if the unit has a target vertex image, remove it
                    self.remove_unit_target_vertex_image()


class menu_button():  # class for creating instances of menu units
    def __init__(self, background, button_text, button_font, button_fg, button_bg, button_xcor, button_ycor, button_func):
        self.button = tk.Label(master = background, text = button_text, font = button_font, fg = button_fg, bg = button_bg)
        self.button.place(x = button_xcor, y = button_ycor, anchor = "center")
        self.button.bind("<Enter>", lambda event, button = self.button: button.configure(font = "{} {} bold".format(button["font"].split(" ")[0], int(button["font"].split(" ")[1]) + 10)))
        self.button.bind("<Leave>", lambda event, button = self.button: button.configure(font=button_font))
        self.button.bind("<Button-1>", lambda event: button_func(event))


class menu_label():  # class for creating instances of menu labels
    def __init__(self, background, label_text, label_font, label_fg, label_bg, label_xcor, label_ycor):
        self.label = tk.Label(master = background, text = label_text, font = label_font, fg = label_fg, bg = label_bg)
        self.label.place(x=label_xcor, y=label_ycor, anchor = "center")


if __name__ == "__main__":
    # windows_number = int(input("How many windows (program instances) do you want to create? "))
    windows_number = 1
    roots_list = []
    networks_list = []
    for window in range(windows_number):
        roots_list.append(tk.Tk())
        networks_list.append(networks_app_window(roots_list[window], window))   
    for window in range(windows_number):
        roots_list[window].mainloop()
