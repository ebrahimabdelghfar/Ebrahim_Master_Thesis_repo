from setuptools import setup

setup(
    name='bayes_race',
    version='1.0',
    author='Achin Jain',
    author_email='achinj@seas.upenn.edu',
    install_requires=[
        'pandas',
        'cvxpy',
        'casadi',
        'botorch',
        'gpytorch',
        'matplotlib',
        'scikit-learn',
        'tikzplotlib'
    ],
)
