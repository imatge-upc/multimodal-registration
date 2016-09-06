import numpy as np
from collections import namedtuple

Test = namedtuple('Test', [
                  'active', 'name', 'paths', 'makes', 'variables', 'runs'])
Run = namedtuple('Run', ['active', 'name', 'command', 'parameters'])

test_files = [
    '/datasets/generations/chessboard.ply',
    '/datasets/generations/chessboard-image.jpg',
]
real_file = ['/results/fullPipeline-kinect/image-cloud.ply']
execs = []

execs.append(Test(
        active=True,
        name="Test synthetic gradients",
        paths={
            'PHD_WORK_PATH': '/results/gradient-synth-canny/',
            'PHD_INPUT_PATH': '/datasets/generations/'

        },
        makes=[
            {'path': 'pcl-build', 'make': 'make'},
            {'path': 'cpp-build', 'make': 'make'}
        ],
        variables={
            'PHD_NORMAL_NOISE': [np.arange(0, 0.2, 0.005), '%0.02f'],
            # 'PHD_COLOR_NOISE': [np.arange(0, 150, 5), '%02d'],
        },
        runs=[
            Run(
                active=True,
                name='generate_cloud',
                command='pcl-build/bin/pcl_mesh_sampling_color' +
                        ' ' + test_files[0] + ' ' +
                        '$PHD_WORK_PATH/chessboard_normal_$PHD_NORMAL_NOISE.pcd',
                parameters={
                    'normal_noise': '$PHD_NORMAL_NOISE',
                    'color_noise': 0,
                    'n_samples': 128*128,
                },
            ),
            Run(
                active=True,
                name='Compute gradient',
                command='cpp-build/gradientEdgePointCloud-multiKcolor',
                parameters={
                    'input': '$PHD_WORK_PATH/chessboard_normal_$PHD_NORMAL_NOISE.pcd',
                    'weight_level_thresh': 10,
                    'weight_level_color_thresh': 0.25,
                    'min_K': 3,
                    'max_K': 20,
                    'max_K_color': 20,
                    'single_k': 0,
                    'step_K': 1,
                    'weight_low_thresh': 0,
                    'output-ply': '$PHD_WORK_PATH/chessboard_normal_$PHD_NORMAL_NOISE-gradient.ply',
                }
            ),
            Run(
                active=True,
                name='Evaluation',
                command='cpp-build/evalDetection',
                parameters={
                    'input-cloud': '$PHD_WORK_PATH/chessboard_normal_$PHD_NORMAL_NOISE-gradient.ply',
                    'input-lines': test_files[0],
                }
            ),
        ]
    ))

execs.append(Test(
        active=True,
        name="Test synthetic gradients",
        paths={
            'PHD_WORK_PATH': '/results/gradient-synth-canny/',
            'PHD_INPUT_PATH': '/datasets/generations/'

        },
        makes=[
            # {'path': 'pcl-build', 'make': 'make'},
            # {'path': 'cpp-build', 'make': 'make'}
        ],
        variables={
            # 'PHD_NORMAL_NOISE': [np.arange(0, 0.1, 0.01), '%0.02f'],
            'PHD_COLOR_NOISE': [np.arange(0, 300, 5), '%02d'],
        },
        runs=[
            Run(
                active=True,
                name='generate_cloud',
                command='pcl-build/bin/pcl_mesh_sampling_color' +
                        ' ' + test_files[0] + ' ' +
                        '$PHD_WORK_PATH/chessboard_color_$PHD_COLOR_NOISE.pcd',
                parameters={
                    'normal_noise': 0,
                    'color_noise': '$PHD_COLOR_NOISE',
                    'n_samples': 10000,
                },
            ),
            Run(
                active=True,
                name='Compute gradient',
                command='cpp-build/gradientEdgePointCloud-multiKcolor',
                parameters={
                    'input': '$PHD_WORK_PATH/chessboard_color_$PHD_COLOR_NOISE.pcd',
                    'weight_level_thresh': 10,
                    'weight_level_color_thresh': 0.25,
                    'min_K': 3,
                    'max_K': 20,
                    'max_K_color': 20,
                    'single_k': 0,
                    'step_K': 1,
                    'weight_low_thresh': 0,
                    'output-ply': '$PHD_WORK_PATH/chessboard_color_$PHD_COLOR_NOISE-gradient.ply',
                }
            ),
            Run(
                active=True,
                name='Evaluation',
                command='cpp-build/evalDetection',
                parameters={
                    'input-cloud': '$PHD_WORK_PATH/chessboard_color_$PHD_COLOR_NOISE-gradient.ply',
                    'input-lines': test_files[0],
                }
            ),
            Run(
                active=True,
                name='Canny',
                command='cpp-build/canny',
                parameters={
                    'input': test_files[1],
                    'noise': '$PHD_COLOR_NOISE',
                    'blur': 0,
                    'output': '$PHD_WORK_PATH/chessboard_normal_$PHD_COLOR_NOISE-canny.ply',
                }
            ),
            Run(
                active=True,
                name='Evaluation',
                command='cpp-build/evalDetection',
                parameters={
                    'input-cloud': '$PHD_WORK_PATH/chessboard_normal_$PHD_COLOR_NOISE-canny.ply',
                    'input-lines': test_files[0],
                }
            ),
            Run(
                active=True,
                name='Canny-blur',
                command='cpp-build/canny',
                parameters={
                    'input': test_files[1],
                    'noise': '$PHD_COLOR_NOISE',
                    'blur': 1,
                    'output': '$PHD_WORK_PATH/chessboard_normal_$PHD_COLOR_NOISE-canny-blur.ply',
                }
            ),
            Run(
                active=True,
                name='Evaluation',
                command='cpp-build/evalDetection',
                parameters={
                    'input-cloud': '$PHD_WORK_PATH/chessboard_normal_$PHD_COLOR_NOISE-canny-blur.ply',
                    'input-lines': test_files[0],
                }
            ),

        ]
    ))
# execs.append(Test(
#         active=True,
#         name="Test synthetic gradients-K",
#         paths={
#             'PHD_WORK_PATH': '/results/gradient-real-K/',
#             'PHD_INPUT_PATH': '/datasets/generations/'

#         },
#         makes=[
#             # {'path': 'pcl-build', 'make': 'make'},
#             # {'path': 'cpp-build', 'make': 'make'}
#         ],
#         variables={
#             'PHD_NORMAL_NOISE': [[0], '%0.02f'],
#             'PHD_COLOR_NOISE': [[0], '%0.01f'],
#             'PHD_K': [range(5, 51), '%02d']
#         },
#         runs=[
#             Run(
#                 active=False,
#                 name='generate_cloud',
#                 command='pcl-build/bin/pcl_mesh_sampling_color' +
#                         ' ' + test_files[0] + ' ' +
#                         '$PHD_INPUT_PATH/chessboard-$PHD_NORMAL_NOISE-$PHD_COLOR_NOISE.pcd',
#                 parameters={
#                     'normal_noise': '$PHD_NORMAL_NOISE',
#                     'color_noise': '$PHD_COLOR_NOISE',
#                     'n_samples': 10000,
#                 },
#             ),
#             Run(
#                 active=False,
#                 name='generate_cloud',
#                 command='pcl-build/bin/pcl_pcd2ply' + ' ' +
#                         '$PHD_INPUT_PATH/chessboard-$PHD_NORMAL_NOISE-$PHD_COLOR_NOISE.pcd' + ' ' +
#                         '$PHD_INPUT_PATH/chessboard-$PHD_NORMAL_NOISE-$PHD_COLOR_NOISE.ply',
#                 parameters={
#                     'format': 0,
#                     'use_camera': 0
#                 },
#             ),
#             Run(
#                 active=True,
#                 name='Compute gradient',
#                 command='cpp-build/gradientEdgePointCloud-multiKcolor',
#                 parameters={
#                     'input': real_file[0],
#                     'weight_level_thresh': 0.05,
#                     'weight_level_color_thresh': 0.25,
#                     'min_K': 3,
#                     'max_K': 150,
#                     'max_K_color': 10,
#                     'single_k': 0,
#                     'step_K': 1,
#                     'weight_low_thresh': 0,
#                     'output-ply': '$PHD_WORK_PATH/chessboard-$PHD_K-gradient.ply',
#                 }
#             ),
#         ]
#     )
# )