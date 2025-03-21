from setuptools import setup, find_packages

setup(
    name='SoloPy',        
    version='4.0.0',      
    license='MIT',      
    description='Python library for interfacing with Solo Motor Controllers',   
    author='Solo',                  
    author_email='support@solomotorcontrollers.com',      
    url='https://github.com/Solo-FL/SoloPy',  
    packages=find_packages(),  # Automatically find all packages
    install_requires=[           
        'pyserial',
        'python-can',
        'python-interface',
    ],
    classifiers=[
        'Development Status :: 4 - Beta',      
        'Intended Audience :: Developers',    
        'Topic :: Software Development :: Build Tools',
        'License :: OSI Approved :: MIT License',   
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    project_urls={  # Replaces deprecated `download_url`
        "Source": "https://github.com/Solo-FL/SoloPy",
        "Download": "https://github.com/Solo-FL/SoloPy/archive/refs/tags/v4.0.0.tar.gz",
        "Documentation": "https://github.com/Solo-FL/SoloPy/wiki",
    },
    python_requires=">=3.4",  # Ensures compatibility with Python 3.4+
)