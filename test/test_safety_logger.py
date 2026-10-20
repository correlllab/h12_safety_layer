'''Tests for safety message output routing'''

import json
import unittest
from unittest.mock import Mock, patch

from h12_safety_layer.core.safety_layer import SafetyLayer
from h12_safety_layer.core.safety_logger import PrintSafetyLogger


class TestSafetyLogger(unittest.TestCase):
    '''Verify plain and injected safety message output'''
    def test_print_logger_flushes_both_levels(self):
        logger = PrintSafetyLogger()
        with patch('builtins.print') as output:
            logger.info('relay started')
            logger.error('estop triggered')

        output.assert_any_call('relay started', flush=True)
        output.assert_any_call('estop triggered', flush=True)

    def test_estop_event_uses_error_and_other_events_use_info(self):
        layer = object.__new__(SafetyLayer)
        layer._status_logger = Mock()

        layer._log_event({'event': 'relay_started', 'mode': 'split_mode'})
        layer._log_event({'event': 'estop_command_written', 'reason': 'limit'})

        layer._status_logger.info.assert_called_once_with(
            json.dumps({'event': 'relay_started', 'mode': 'split_mode'}, sort_keys=True)
        )
        layer._status_logger.error.assert_called_once_with(
            json.dumps({'event': 'estop_command_written', 'reason': 'limit'}, sort_keys=True)
        )


if __name__ == '__main__':
    unittest.main()
