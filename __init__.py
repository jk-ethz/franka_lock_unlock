#!/usr/bin/env python3
# Copyright jk-ethz
# Released under GNU AGPL-3.0
# Contact us for other licensing options.

# Developed and tested on system version
# 4.2.1

# Inspired by
# https://github.com/frankaemika/libfranka/issues/63
# https://github.com/ib101/DVK/blob/master/Code/DVK.py

import requests
from urllib.parse import urljoin
import hashlib
import base64
import argparse
from time import sleep
from itertools import count
import atexit
from threading import Event

class FrankaLockUnlock:
    def __init__(self, hostname: str, username: str, password: str, protocol: str = 'https', relock: bool = False):
        requests.packages.urllib3.disable_warnings()
        self._session = requests.Session()
        self._session.verify = False
        self._hostname = f'{protocol}://{hostname}'
        self._username = username
        self._password = password
        self._logged_in = False
        self._token = None
        self._token_id = None
        self._relock = relock
        atexit.register(self._cleanup)

    def _cleanup(self):
        print("Cleaning up...")
        if self._relock:
            self.run(unlock=False)
        if self._token is not None or self._token_id is not None:
            self._release_token()
        if self._logged_in:
            self._logout()
        print("Successfully cleaned up.")

    @staticmethod
    def _encode_password(username, password):
        bs = ','.join([str(b) for b in hashlib.sha256((f'{password}#{username}@franka').encode('utf-8')).digest()])
        return base64.encodebytes(bs.encode('utf-8')).decode('utf-8')

    def _login(self):
        print("Logging in...")
        if self._logged_in:
            print("Already logged in.")
            return
        login = self._session.post(urljoin(self._hostname, '/admin/api/login'), \
                                           json={'login': self._username, \
                                                 'password': self._encode_password(self._username, self._password)})
        assert login.status_code == 200, "Error logging in."
        self._session.cookies.set('authorization', login.text)
        self._logged_in = True
        print("Successfully logged in.")

    def _logout(self):
        print("Logging out...")
        assert self._logged_in
        logout = self._session.post(urljoin(self._hostname, '/admin/api/logout'))
        assert logout.status_code == 200, "Error logging out"
        self._session.cookies.clear()
        self._logged_in = False
        print("Successfully logged out.")

    def _get_active_token_id(self):
        token_query = self._session.get(urljoin(self._hostname, '/admin/api/control-token'))
        assert token_query.status_code == 200, "Error getting control token status."
        json = token_query.json()
        return None if json['activeToken'] is None else json['activeToken']['id']

    def _is_active_token(self):
        active_token_id = self._get_active_token_id()
        return active_token_id is None or active_token_id == self._token_id

    def _request_token(self, physically=False):
        print("Requesting a control token...")
        if self._token is not None:
            assert self._token_id is not None
            print("Already having a control token.")
            return
        token_request = self._session.post(urljoin(self._hostname, f'/admin/api/control-token/request{"?force" if physically else ""}'), \
                                           json={'requestedBy': self._username})
        assert token_request.status_code == 200, "Error requesting control token."
        json = token_request.json()
        self._token = json['token']
        self._token_id = json['id']
        print(f'Received control token is {self._token} with id {self._token_id}.')

    def _release_token(self):
        print("Releasing control token...")
        token_delete = self._session.delete(urljoin(self._hostname, '/admin/api/control-token'), \
                                                    json={'token': self._token})
        assert token_delete.status_code == 200, "Error releasing control token."
        self._token = None
        self._token_id = None
        print("Successfully released control token.")

    def _activate_fci(self):
        print("Activating FCI...")
        fci_request = self._session.post(urljoin(self._hostname, f'/admin/api/control-token/fci'), \
                                         json={'token': self._token})
        assert fci_request.status_code == 200, "Error activating FCI."
        print("Successfully activated FCI.")

    def _lock_unlock(self, unlock: bool, force: bool = False):
        print(f'{"Unlocking" if unlock else "Locking"} the robot...')
        action = self._session.post(urljoin(self._hostname, f'/desk/api/robot/{"open" if unlock else "close"}-brakes'), \
                                                        files={'force': force},
                                                        headers={'X-Control-Token': self._token})
        assert action.status_code == 200, "Error requesting brake open/close action."
        print(f'Successfully {"unlocked" if unlock else "locked"} the robot.')

    def run(self, unlock: bool = False, force: bool = False, wait: bool = False, request: bool = False, persistent: bool = False, fci: bool = False) -> None:
        assert not request or wait, "Requesting control without waiting for obtaining control is not supported."
        assert not fci or unlock, "Activating FCI without unlocking is not possible."
        assert not fci or persistent, "Activating FCI without persistence is not possible."
        self._login()
        try:
            assert self._token is not None or self._get_active_token_id() is None or wait, "Error requesting control, the robot is currently in use."
            while True:
                self._request_token(physically=request)
                try:
                    # Consider the timeout of 30 s for requesting physical access to the robot
                    for _ in range(20) if request else count():
                        if (not wait and not request) or self._is_active_token():
                            print('Successfully acquired control over the robot.')
                            self._lock_unlock(unlock=unlock)
                            if fci:
                                self._activate_fci()
                            return
                        if request:
                            print('Please press the button with the (blue) circle on the robot to confirm physical access.')
                        elif wait:
                            print('Please confirm the request message in the web interface on the logged in user.')
                        sleep(1)
                    # In case physical access was not confirmed, try again
                    self._release_token()
                finally:
                    if not persistent:
                        self._release_token()
        finally:
            if not persistent:
                self._logout()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                                     prog = 'FrankaLockUnlock',
                                     description = 'Lock or unlock the Franka Emika Panda joint brakes programmatically.',
                                     epilog = '(c) jk-ethz, https://github.com/jk-ethz'
                                    )
    parser.add_argument('hostname', help='The Franka Desk IP address or hostname, for example "1.2.3.4".')
    parser.add_argument('username', help='The Franka Desk username, usually "admin".')
    parser.add_argument('password', help='The Franka Desk password.')
    parser.add_argument('-u', '--unlock', action='store_true', help='Unlock the brakes. Otherwise, lock them.')
    parser.add_argument('-l', '--relock', action='store_true', help='Relock the brakes on exit.')
    parser.add_argument('-w', '--wait', action='store_true', help='Wait in case the robot web UI is currently in use.')
    parser.add_argument('-r', '--request', action='store_true', help='Request control by confirming physical access to the robot in case the robot web UI is currently in use.')
    parser.add_argument('-p', '--persistent', action='store_true', help='Keep the connection to the robot open persistently.')
    parser.add_argument('-c', '--fci', action='store_true', help='Activate the FCI.')
    args, _ = parser.parse_known_args()
    assert not args.relock or args.unlock, "Relocking without prior unlocking is not possible."
    assert not args.relock or args.persistent, "Relocking without persistence would cause an immediate unlock-lock cycle."

    franka_lock_unlock = FrankaLockUnlock(hostname=args.hostname, username=args.username, password=args.password, relock=args.relock)
    franka_lock_unlock.run(unlock=args.unlock, wait=args.wait, request=args.request, persistent=args.persistent, fci=args.fci)

    if args.persistent:
        print("Keeping persistent connection...")
        Event().wait()
