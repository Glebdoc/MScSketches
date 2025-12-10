import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin

# Function to get .dat files from a website
def download_dat_files(url, save_folder='dat_files'):
    
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)

    # Fetch the website content
    response = requests.get(url)
    if response.status_code != 200:
        print(f"Failed to retrieve the webpage: {response.status_code}")
        return

    # Parse the HTML content
    soup = BeautifulSoup(response.text, 'html.parser')

    # Find all links on the webpage
    links = soup.find_all('a', href=True)

    # Filter for links that end with .dat
    dat_files = [link['href'] for link in links if link['href'].endswith('.dat')]

    if not dat_files:
        print("No .dat files found on the webpage.")
        return

    # Download each .dat file
    for dat_file in dat_files:
        file_url = urljoin(url, dat_file)
        file_name = os.path.join(save_folder, os.path.basename(dat_file))

        print(f"Downloading {file_name}...")

        # Download the file
        file_response = requests.get(file_url)
        if file_response.status_code == 200:
            with open(file_name, 'wb') as f:
                f.write(file_response.content)
            print(f"Saved: {file_name}")
        else:
            print(f"Failed to download {file_url}: {file_response.status_code}")

# Example usage
website_url = 'https://m-selig.ae.illinois.edu/ads/coord_database.html'
download_dat_files(website_url)
