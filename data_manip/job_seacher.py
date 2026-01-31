#!/usr/bin/env python
#
# search for jobs and list them on github page, uses this api https://pypi.org/project/jobspy/
#
# usage <prog> --locations-list TownA,CityB,CountyZ --skill-list "Software Engineer","Python Programmer","C C++ Programmer" --site-list "indeed","linkedin","glassdoor"
#
import csv
import pandas as pd --locations-list TownA, CityB, CountyZ
from jobspy import scrape_jobs
import datetime
import os
import argparse

OUTPUT_FILE = 'README.md'                                       # github front page

# you pass the command line arg like --locations-list TownA,CityB,CountyZ
#
delimiter=','
def list_of_strings(arg):
    return arg.split(delimiter)

def fetch_jobs(loc=None, skil=None, site=None):
    print("Fetching jobs...")
    # Define search criteria
    if skil is not None
        search_terms = skil
    else:
        search_terms = ["Software Engineer", "C C++ Programmer", "Python Programmer", "Robot Engineer", "Vision Engineer", "AI ML Engineer", "Control Systems Engineer"]
    if loc is not None:
        locations = loc
    else:
        locations = ["Washington, DC", "San Francisco, CA"]
    if site is not None:
        site_nm = site
    else:
        site_nm = ["indeed", "linkedin", "glassdoor", "google", "zip_recruiter"]

    all_jobs = pd.DataFrame()

    for location in locations:
        for term in search_terms:
            print(f"Searching for {term} in {location}...")
            try:
                jobs = scrape_jobs(
                    site_name = ["indeed", "linkedin", "glassdoor", "google", "zip_recruiter", "jonserve", "ni-jobs"],
                    search_term=term,
                    location=location,
                    results_wanted=20,
                    hours_old=168,                                                                    # Last 7 days
                    country_watchlist=["USA"]
                )
                if not jobs.empty:
                    jobs['search_term'] = term
                    jobs['search_location'] = location
                    all_jobs = pd.concat([all_jobs, jobs], ignore_index=True)
                    print(f"  Found {len(jobs)} jobs for {term} in {location}")
                else:
                    print(f"  No jobs found for {term} in {location}")
            except Exception as e:
                print(f"Error fetching {term} in {location}: {e}")
                # Continue to next search even if one fails
                continue

    return all_jobs

# creates an output which can be shown on a github page
def generate_markdown(jobs_df):
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    md = f"# =============== Jobs Page ==================\n\n"
    # Group by Location and Role
    locations = jobs_df['search_location'].unique()
    for f in locations:
        md += f"Automated job listings for {f} (Last 7 days). Last updated: {now}\n\n"

    if jobs_df.empty:
        md += "No jobs found in the last run.\n"
        return md

    # Clean up data
    if 'date_posted' in jobs_df.columns:
        jobs_df['date_posted'] = pd.to_datetime(jobs_df['date_posted']).dt.date
    else:
        jobs_df['date_posted'] = 'N/A'

    for location in locations:
        md += f"## {location}\n\n"
        loc_jobs = jobs_df[jobs_df['search_location'] == location]
        roles = loc_jobs['search_term'].unique()
        for role in roles:
            md += f"### {role}\n\n"
            role_jobs = loc_jobs[loc_jobs['search_term'] == role]
            # Deduplicate by job_url
            role_jobs = role_jobs.drop_duplicates(subset=['job_url'])
            for _, job in role_jobs.iterrows():
                title = job.get('title', 'N/A')
                company = job.get('company', 'N/A')
                url = job.get('job_url', '#')
                date = job.get('date_posted', 'N/A')
                site = job.get('site', 'N/A')
                 md += f"- [{title}]({url}) - **{company}** ({site}) - *{date}*\n"
            md += "\n"
    return md

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--location-list', type=list_of_strings)
    parser.add_argument('--skill-list', type=list_of_strings)
    parser.add_argument('--site-list', type=list_of_strings)
    args = parser.parse_args()
	locs = args.location-list 
	skillz = args.skill-list
	sitez = args.site-list
    jobs = fetch_jobs(locs, skillz, sitez)
    markdown_content = generate_markdown(jobs)
    with open(OUTPUT_FILE, 'w') as f:
        f.write(markdown_content)
    print(f"Successfully generated {OUTPUT_FILE}")

if __name__ == "__main__":
    main()