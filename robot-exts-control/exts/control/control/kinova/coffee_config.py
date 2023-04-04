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

    "'approach_coffee_machine_handle'": {
        'base_prim': '/coffee_machine',
        'action_type': 'move',
        'transform_offset': {
            'position': [0.5, 0.0, 0.0],
            'orientation': [1.0, 0.0, 0.0, 0.0],
        }
    
    }
}