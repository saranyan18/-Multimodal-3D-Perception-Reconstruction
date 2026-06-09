import os
import sys
import urllib.request
import zipfile


def show_progress(blocknum, blocksize, totalsize):
    readsofar = blocknum * blocksize
    if totalsize > 0:
        percent = readsofar * 100 / totalsize
        progress_str = f"\rProgress: {percent:.1f}% ({readsofar / 1e6:.1f} MB / {totalsize / 1e6:.1f} MB)"
        sys.stdout.write(progress_str)
        sys.stdout.flush()
    else:
        sys.stdout.write(f"\rDownloaded {readsofar / 1e6:.1f} MB")

urls = {
    "calib.zip": "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip",
    "sync.zip": "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0023/2011_09_26_drive_0023_sync.zip",
    "tracklets.zip": "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0023/2011_09_26_drive_0023_tracklets.zip"
}


os.makedirs('./data', exist_ok=True)


for filename, url in urls.items():
    print(f"\n--- Starting download for {filename} ---")
    try:

        urllib.request.urlretrieve(url, filename, show_progress)

        print(f"\nExtracting {filename} into the './data' folder...")
        with zipfile.ZipFile(filename, 'r') as zip_ref:
            zip_ref.extractall('./data')
            
  
        os.remove(filename)
        print(f"Finished processing {filename} successfully.")
        
    except Exception as e:
        print(f"\nAn error occurred while processing {filename}: {e}")

print("\n[SUCCESS] Everything is downloaded and structured perfectly inside your './data' folder!")