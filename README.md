# Roverside HEBI Claw Manipulator Package

Roverside Assistance project HEBI claw manipulator package provides claw
manipulator control utilities for the rovers with HEBI actuators.

Setup and installation process (for commit hash code - 5df4e76):

1. Clone this repository
2. Visit: http://docs.hebi.us/tools.html#Tools_APIs
3. Check the python Tool chai (link: https://pypi.python.org/pypi/hebi-py)
4. Download the hebi package
5. extact the tar ball (tar -xf hebi-py-*.tar.gz)
6. cd Hebi-py-*
7. Build the package "python setup.py build".
Check python setup.py --help for more options
8. Install the package (requires admin. persmissions) "sudo python setup.py install"
9. All done! - installation complete
10. An external test would be the following:
10.1. ipython
10.2. in the ipython terminal "import hebi" - if successful - installation is a success. 
11. Migrate to the roverside_hebi package (cloned above).
12. Further to the test "cd test"
13. run "./hebi_test.py"
