from typing import Optional

from pdfminer.pdfparser import PDFParser
from pdfminer.pdfdocument import PDFDocument


class PdfInfo:
    def __init__(self, filename: str):
        with open(filename, "rb") as pdf:
            parser = PDFParser(pdf)
            self.__doc = PDFDocument(parser)
            parser.set_document(self.__doc)

            self.__info = self.__doc.info[0]

    def get_author(self):
        if "Author" in self.__info:
            return PdfInfo.__decode(self.__info["Author"])
        else:
            return None

    def get_title(self):
        if "Title" in self.__info:
            return PdfInfo.__decode(self.__info["Title"])
        else:
            return None

    @staticmethod
    def __decode(byte_string: bytes) -> Optional[str]:
        for c in ["utf-8", "cp1252"]:
            try:
                return byte_string.decode(c)
            except UnicodeDecodeError:
                pass

        return None