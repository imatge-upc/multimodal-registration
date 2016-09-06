import numpy as np
from collections import namedtuple

Test = namedtuple('Test', [
                  'active', 'name', 'paths', 'makes', 'variables', 'runs'])
Run = namedtuple('Run', ['active', 'name', 'command', 'parameters'])

test_files = [
    '/datasets/bunny/stanford-bunny-cebal-ssh.jpg',
    '/datasets/bunny/reconstruction/bun_zipper.ply',
]


execs = []
execs.append(Test(
        active=True,
        name="Test full pipeline",
        paths={
            'PHD_WORK_PATH': '/results/fullPipeline-kinect-bunny/'
        },
        makes=[
            # {'path': 'pcl-build', 'make': 'make'},
            {'path': 'cpp-build', 'make': 'make'}
        ],
        variables={
            'PHD_THRESHOLD': [[0], '%0.02f'],
        },
        runs=[
            Run(
                active=True,
                name='0. Convert image to point cloud (image)',
                command='cpp-build/image2cloud',
                parameters={
                    'input': test_files[0],
                    'scale': 1,
                    'dist_coeffs': 0,
                    'cx_cy_fx_fy': '\'400.0 378.0 10000 10000\'',
                    'output-ply': '$PHD_WORK_PATH/image-cloud.ply',
                }
            ),
            Run(
                active=True,
                name='0. Convert image to point cloud (image)',
                command='cpp-build/flatten_cloud',
                parameters={
                    'input': test_files[1],
                    'output': '$PHD_WORK_PATH/image-cloud.ply',
                }
            ),
            Run(
                active=True,
                name='1. Computing image gradient (image)',
                command='cpp-build/gradientEdgePointCloud-multiKcolor',
                parameters={
                    'input': '$PHD_WORK_PATH/image-cloud.ply',
                    'weight_level_thresh': 20,
                    'weight_level_color_thresh': 0.01,
                    'min_K': 3,
                    'max_K': 10,
                    'max_K_color': 10,
                    'single_k': 0,
                    'step_K': 1,
                    'weight_low_thresh': 0,
                    'output-ply': '$PHD_WORK_PATH/gradient-image.ply',
                }
            ),
            Run(
                active=True,
                name='1. Computing cloud gradient (cloud)',
                command='cpp-build/gradientEdgePointCloud-multiKcolor',
                parameters={
                    'input': test_files[1],
                    'weight_level_thresh': 0.02,
                    'weight_level_color_thresh': 1,
                    'min_K': 3,
                    'max_K': 100,
                    'max_K_color': 1,
                    'single_k': 0,
                    'step_K': 1,
                    'weight_low_thresh': 0,
                    'output-ply': '$PHD_WORK_PATH/gradient-cloud.ply',
                }
            ),
            Run(
                active=True,
                name='2. Hystheresis thresholding (image)',
                command='cpp-build/hystheresisThresholding',
                parameters={
                    'input': '$PHD_WORK_PATH/gradient-image.ply',
                    'low_thresh': 0.4,
                    'high_thresh': 0.6,
                    'output-ply': '$PHD_WORK_PATH/gradient-hystheresis-image.ply',
                }
            ),
            Run(
                active=True,
                name='2. Hystheresis thresholding (cloud)',
                command='cpp-build/hystheresisThresholding',
                parameters={
                    'input': '$PHD_WORK_PATH/gradient-cloud.ply',
                    'low_thresh': 0.4,
                    'high_thresh': 0.6,
                    'output-ply': '$PHD_WORK_PATH/gradient-hystheresis-cloud.ply',
                }
            ),
            Run(
                active=True,
                name='3. Register - hystheresis',
                command='cpp-build/featureMatching-multimodal',
                parameters={
                    'cloud': '$PHD_WORK_PATH/gradient-hystheresis-cloud.ply',
                    'image': '$PHD_WORK_PATH/gradient-hystheresis-image.ply',
                    'cloud_full': test_files[1],
                    'final_image': '$PHD_WORK_PATH/final_image-hist.ply',
                    'final_cloud': '$PHD_WORK_PATH/final_cloud-hist.ply',
                    'final_cloud_projected': '$PHD_WORK_PATH/final_cloud_projected-hist.ply',
                    'rt': "\'0 0.1 -1 0 0 0\'"
                }
            ),
        ]
    ))
