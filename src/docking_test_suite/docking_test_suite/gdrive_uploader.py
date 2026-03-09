"""Google Drive upload utility for the docking test suite.

Setup
-----
1. Go to https://console.cloud.google.com/
2. Create a project (or use an existing one).
3. Enable the **Google Drive API**.
4. Under *Credentials*, create an **OAuth 2.0 Client ID** (Desktop app).
5. Download the JSON file and save it as the path specified in
   ``google_drive.credentials_file`` in your ``test_config.yaml``
   (default: ``~/docking_test_data/credentials.json``).
6. Set ``google_drive.enabled: true`` and fill in ``folder_id``
   (the alphanumeric ID from the Google Drive folder URL).
7. The first time you upload, a browser window will open for
   authentication.  The resulting token is cached in ``token_file``.

Dependencies
------------
.. code-block:: bash

   pip install google-api-python-client google-auth-httplib2 google-auth-oauthlib
"""

import os
import glob


def _get_drive_service(cfg: dict):
    """Build and return an authenticated Google Drive API service object."""
    creds_file = os.path.expanduser(cfg['google_drive']['credentials_file'])
    token_file = os.path.expanduser(cfg['google_drive']['token_file'])

    if not os.path.isfile(creds_file):
        raise FileNotFoundError(
            f'Google Drive credentials file not found: {creds_file}\n'
            'See docking_test_suite/gdrive_uploader.py docstring for setup instructions.')

    from google.oauth2.credentials import Credentials
    from google_auth_oauthlib.flow import InstalledAppFlow
    from google.auth.transport.requests import Request
    from googleapiclient.discovery import build

    SCOPES = ['https://www.googleapis.com/auth/drive.file']
    creds = None

    if os.path.isfile(token_file):
        creds = Credentials.from_authorized_user_file(token_file, SCOPES)

    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(creds_file, SCOPES)
            creds = flow.run_local_server(port=0)
        with open(token_file, 'w') as tok:
            tok.write(creds.to_json())

    return build('drive', 'v3', credentials=creds)


def upload_file(cfg: dict, local_path: str, logger=None) -> str:
    """Upload a single file to Google Drive.

    Returns the Google Drive file ID.
    """
    from googleapiclient.http import MediaFileUpload

    service = _get_drive_service(cfg)
    folder_id = cfg['google_drive'].get('folder_id', '')

    file_metadata = {'name': os.path.basename(local_path)}
    if folder_id:
        file_metadata['parents'] = [folder_id]

    media = MediaFileUpload(local_path, resumable=True)
    result = service.files().create(
        body=file_metadata, media_body=media, fields='id').execute()

    file_id = result.get('id', '')
    if logger:
        logger.info(f'Uploaded {local_path} -> Google Drive ID: {file_id}')
    else:
        print(f'Uploaded {local_path} -> Google Drive ID: {file_id}')
    return file_id


def upload_directory(cfg: dict, local_dir: str, logger=None):
    """Upload all files in a directory to Google Drive."""
    if not os.path.isdir(local_dir):
        return

    files = sorted(glob.glob(os.path.join(local_dir, '*')))
    for f in files:
        if os.path.isfile(f):
            upload_file(cfg, f, logger)


def upload_test_results(cfg: dict, csv_path: str, logger=None):
    """Upload the CSV and any associated image directory."""
    upload_file(cfg, csv_path, logger)

    # Check for companion image directory
    base = os.path.splitext(csv_path)[0]
    img_dir = base + '_images'
    if os.path.isdir(img_dir):
        upload_directory(cfg, img_dir, logger)


def maybe_upload(cfg: dict, csv_path: str, logger=None):
    """Upload if Google Drive is enabled and auto_upload is on."""
    gd = cfg.get('google_drive', {})
    if not gd.get('enabled', False):
        return
    if not gd.get('auto_upload', False):
        return
    try:
        upload_test_results(cfg, csv_path, logger)
    except Exception as e:
        msg = f'Google Drive upload failed: {e}'
        if logger:
            logger.warn(msg)
        else:
            print(f'WARNING: {msg}')


# ---- CLI entry point ----

def main():
    """Manual upload tool: upload a file or directory to Google Drive."""
    import argparse
    parser = argparse.ArgumentParser(description='Upload test data to Google Drive')
    parser.add_argument('path', help='File or directory to upload')
    parser.add_argument('--config', default=None, help='Path to test_config.yaml')
    args = parser.parse_args()

    from .config_loader import load_config
    cfg = load_config(args.config)

    if not cfg.get('google_drive', {}).get('enabled', False):
        print('Google Drive is disabled in config. Set google_drive.enabled: true')
        return

    path = os.path.expanduser(args.path)
    if os.path.isfile(path):
        upload_file(cfg, path)
    elif os.path.isdir(path):
        upload_directory(cfg, path)
    else:
        print(f'Path not found: {path}')


if __name__ == '__main__':
    main()
