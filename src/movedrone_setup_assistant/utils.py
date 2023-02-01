import rospkg


def get_pkg_path() -> str:
    """ スクリプトのパッケージまでのフルパスを返す． """

    pkg_name = rospkg.get_package_name(__file__)
    return rospkg.RosPack().get_path(pkg_name)
