from distutils.core import setup
setup(
  name = 'SoloPy',        
  packages = ['SoloPy'],  
  version = 'v3.1.3',      
  license='gpl-3.0',      
  description = 'PYTHON Library for Control Solo Motor',   
  author = 'Solo',                  
  author_email = 'support@solomotorcontrollers.com',      
  url = 'https://github.com/Solo-FL/SoloPy',   
  download_url = 'https://github.com/Solo-FL/SoloPy/archive/refs/tags/v3.1.3.tar.gz',    
  keywords = ['Solo Motor controllers', 'Solo', 'SoloPy'],   
  install_requires=[           
          'pyserial==3.5',
          'python-can==4.2.2',
          'python-interface==1.6.1'
      ],
  classifiers=[
    'Development Status :: 4 - Beta',      
    'Intended Audience :: Developers',    
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',   
    'Programming Language :: Python :: 3',      
    'Programming Language :: Python :: 3.4',
    'Programming Language :: Python :: 3.5',
    'Programming Language :: Python :: 3.6',
    'Programming Language :: Python :: 3.7',
    'Programming Language :: Python :: 3.8',
    'Programming Language :: Python :: 3.9',
  ],
)
