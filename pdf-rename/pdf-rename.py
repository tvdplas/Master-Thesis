from argparse import ArgumentParser
from ast import Interactive
import os

from sanitize_filename import sanitize

from pdf_parser import PdfInfo

def main():
    parser = ArgumentParser(description="Rename pdf files in a folder")
    parser.add_argument("folder", metavar="Folder", type=str)
    parser.add_argument("-n", dest="dry_run", action="store_true")
    parser.add_argument("-i", dest="interactive", action="store_true")

    args = parser.parse_args()
    for root, _dirs, files in os.walk(args.folder):
        for file in files:
            if file[-4:] != ".pdf":
                continue

            file_info = PdfInfo(os.path.join(root, file))

            author = file_info.get_author()
            title = file_info.get_title()

            new_filename = ""
            if author is not None and title is not None:
                author_string = author.split(';')[0].split(',')[0].split(' ')[-1]

                new_filename = f"{author_string} - {title}.pdf"
            elif title is not None:
                new_filename = f"{title}.pdf"
            else:
                print("No info - Skipping")
                continue

            new_filename = sanitize(new_filename)
            if file == new_filename:
                continue

            if args.dry_run:
                print(f"Would rename: {file} -> {new_filename}")
                continue
            if args.interactive:
                while True:
                    a = input(f"Rename: {file} -> {new_filename}? [Y/N] ")
                    if a == 'Y' or a == 'N':
                        break

                if a == 'N':
                    continue

            # Actual renaming
            print(f"Renaming: {file} -> {new_filename}")
            os.rename(os.path.join(root, file), os.path.join(root, new_filename))
            print()


if __name__ == "__main__":
    main()
