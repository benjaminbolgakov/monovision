from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='thesis_vision',
    version='0.1.0',
    description='Computer vision thesis',
    long_description=readme,
    author='Benjamin Bolgakov & Anton Frank',
    author_email='benjaminbolgakov@gmail.com',
    url='https://github.com/benjaminbolgakov/thesis_vision',
    license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)
