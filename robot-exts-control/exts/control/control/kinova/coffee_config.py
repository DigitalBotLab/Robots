# task config for making coffee

kinova_action_config = {
    "go_home": {
        'base_prim': None,
        'steps':[
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0.45666, 0.0, 0.43371],
                'orientation': [0.5, 0.5, 0.5, 0.5], # wxyz
            }
        ]
    },
    "pick_up_capsule": {
        'base_prim': '/World/k_cup',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [-0.12, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.12, 0.0, 0.1],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.12, 0.0, 0.03],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 100,
                'ratio': 0.6,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [-0.12, 0.0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
        ]
    },

    "pick_up_papercup": {
        'base_prim': '/World/papercup',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [-0.3, 0, 0.3],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [-0.3, 0, 0.05],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [-0.15, 0, 0.05],
                'orientation': [0.5, 0.5, 0.5, 0.5],
            },
        ]
    },

    "open_coffee_machine_handle": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.4, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.29, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 100,
                'ratio': 0.8,
            },
            # {
            #     'action_type': 'slerp',
            #     'duration': 300,
            #     'sub_steps': 10,
            #     'position': [0, -0.28, 0],
            #     'orientation': [-0.5, 0.5, 0.5, 0.5],
            #     'relative_rotation': [0.7372773, -0.6755902, 0, 0],
                # 'slerp_last': True,
                #     'slerp_offset': [0, -0.001, 0]
            # },
            # {
            #     'action_type': 'close', #open
            #     'duration': 100,
            #     'ratio': 0.05,
            # },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0.3, -0.5, 0.3],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
           
        ]
    },

    "move_capsule_to_coffee_machine": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.3, 0.025],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.18, 0.025],
                'orientation':  [0, 0, 0.7071, 0.7071],
            },     
            {
                'action_type': 'close', #open
                'duration': 100,
                'ratio': 0.05,
            },      
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.3, 0.025],
                'orientation': [0, 0, 0.7071, 0.7071],
            },
        ]
    },

    "close_coffee_machine_handle": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'close', #open
                'duration': 30,
                'ratio': 0.95,
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, 0, 0.25],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
            {
                'action_type': 'slerp',
                'duration': 140,
                'sub_steps': 7,
                'position': [0, 0, 0.25],
                'orientation': [0, 0.7071, 0.7071, 0],
                'relative_rotation': [0.7660444, 0.6427876, 0, 0],
                'slerp_last': False,
                'slerp_offset': [0, 0, 0]
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.4, 0.1],
                'orientation': [-0.4545, 0.5416,  0.5416, 0.4545],
            },
        ]
    },
    "press_coffee_machine_button": {
        'base_prim': '/World/Keurig_1_5_add_hold/XformHandle',
        'steps':[
            {
                'action_type': 'close', #open
                'duration': 30,
                'ratio': 0.95,
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.1, 0.2],
                'orientation': [0, 0.7071, 0.7071, 0],
            },
           
        ]
    },
}