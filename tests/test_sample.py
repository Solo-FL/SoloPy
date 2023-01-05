# Copyright: (c) 2021, 2022, 2023 SOLO motor controllers project
# GNU General Public License v3.0+ (see COPYING or https://www.gnu.org/licenses/gpl-3.0.txt)

# Title: SoloPy
# Author: SOLOMotorControllers
# Date: 2023
# Code version: 3.0.0
# Availability: https://github.com/Solo-FL/SoloPy/tree/main/SoloPy
# This Library is made by SOLOMotorControllers.COM
# please visit:  https://www.SOLOMotorControllers.com/

#https://docs.pytest.org/en/7.1.x/getting-started.html
def func(x):
    return x + 1


def test_answer():
    assert func(3) == 5 #give error
    #assert func(3) == 4 #give succes


#RUN THE TEST
#pytest test_sample.py

#RUN THE TEST SYNTETIC LOG
#pytest -q test_sample.py