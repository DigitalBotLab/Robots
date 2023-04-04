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
        'base_prim': '/World/capsule',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, 0.3, -0.15],
                'orientation': [1.0, 0.0, 0.0, 0.0],
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, 0.03, -0.15],
                'orientation': [1.0, 0.0, 0.0, 0.0],
            },
            {
                'action_type': 'close',
                'duration': 100,
                'gripper_ratio': 100,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, 0.2, -0.15],
                'orientation': [1.0, 0.0, 0.0, 0.0],
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, 0.2, -0.15],
                'orientation': [0.0, 0.0, 0.0, 1.0],
            },

        ]
    },

    "open_coffee_machine_handle": {
        'base_prim': '/World/Keurig_1_5_add_hold',
        'steps':[
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.5, 0.30921],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'move',
                'duration': 200,
                'position': [0, -0.330, 0.30921],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 100,
                'gripper_ratio': 100,
            },
            {
                'action_type': 'move',
                'duration': 100,
                'position': [0, -0.330, 0.4],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
           
        ]
    },
}