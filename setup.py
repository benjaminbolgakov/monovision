from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='monovision',
    version='0.1.0',
    description='Monocular SLAM solution',
    long_description=readme,
    author='Benjamin Bolgakov',
    author_email='benjaminbolgakov@gmail.com',
    url='https://github.com/benjaminbolgakov/monovision.git',
    license=license,
    packages=find_packages(exclude=('monov'))
)
