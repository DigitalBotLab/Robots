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
        'base_prim': '/World/Keurig_1_5_add_hold/Handle',
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
                'position': [0, -0.28, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
            },
            {
                'action_type': 'close',
                'duration': 100,
                'gripper_ratio': 100,
            },
            {
                'action_type': 'slerp',
                'duration': 600,
                'sub_steps': 10,
                'position': [0, -0.28, 0],
                'orientation': [-0.5, 0.5, 0.5, 0.5],
                'relative_rotation': [0.7071, -0.7071, 0, 0],
            },
           
        ]
    },
}