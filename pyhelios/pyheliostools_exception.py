class PyHeliosToolsException(Exception):
    """PyHelios tools exception class"""

    def __init__(self, msg):
        """PyHeliosToolsException constructor: Build a new exception.

        Arguments:
            msg -- message for the exception
        """
        super(Exception, self).__init__(msg)
