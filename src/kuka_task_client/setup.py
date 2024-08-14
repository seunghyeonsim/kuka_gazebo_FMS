from setuptools import setup

package_name = 'kuka_task_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'rclpy', 'numpy', 'scipy'],  # 종속성 업데이트
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='example@domain.com',
    description='Python client for Kuka transform input service',
    license='Apache-2.0',  # 라이센스 형식 수정
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_client = kuka_task_client.task_client:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
