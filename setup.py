from distutils.core import setup
setup(
  name = 'SoloPy',         # How you named your package folder (MyLib)
  packages = ['SoloPy'],   # Chose the same as "name"
  version = 'v1.0',      # Start with a small number and increase it with every change you make
  license='gpl-3.0',        # Chose a license from here: https://help.github.com/articles/licensing-a-repository
  description = 'PYTHON Library for Control Solo Motor',   # Give a short description about your library
  author = 'Solo',                   # Type in your name
  author_email = 'support@solomotorcontrollers.com',      # Type in your E-Mail
  url = 'https://github.com/Solo-FL/SoloPy',   # Provide either the link to your github or to your website
  download_url = 'https://github.com/Solo-FL/SoloPy/archive/refs/tags/v1.0.tar.gz',    # I explain this later on
  keywords = ['Solo Motor controllers', 'Solo', 'SoloPy'],   # Keywords that define your package best
  install_requires=[            # I get to this in a second
          'pyserial'
      ],
  classifiers=[
    'Development Status :: 4 - Beta',      # Chose either "3 - Alpha", "4 - Beta" or "5 - Production/Stable" as the current state of your package
    'Intended Audience :: Developers',      # Define that your audience are developers
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',   # Again, pick a license
    'Programming Language :: Python :: 3',      #Specify which pyhton versions that you want to support
    'Programming Language :: Python :: 3.4',
    'Programming Language :: Python :: 3.5',
    'Programming Language :: Python :: 3.6',
    'Programming Language :: Python :: 3.7',
    'Programming Language :: Python :: 3.8',
    'Programming Language :: Python :: 3.9',
  ],
)
