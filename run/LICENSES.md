# Licenses for third-party modules/software
This document lists the licenses for the external modules/dependencies included 
with the binary distribution of HELIOS++. We have compiled this list to the best 
of our knowledge, but if you think attribution of a module is missing, please contact us: 
HELIOS++ development team <helios@uni-heidelberg.de>.

License texts follow below the list of third-party software.

Below follows a list of software, license, links to download the source code, and the respective DLLs:
* scilab (GPLv2, https://www.scilab.org/download/6.1.0)
  * `api_scilab.dll`
  * `blasplus.dll`
  * `ast.dll`
  * `boolean.dll`
  * `boolean_gw.dll`
  * `cacsd.dll`
  * `cacsd_f.dll`
  * `call_scilab.dll`
  * `console_gw.dll`
  * `core.dll`
  * `core_f.dll`
  * `core_gw.dll`
  * `coverage.dll`
  * `data_structures.dll`
  * `differential_equations.dll`
  * `differential_equations_f.dll`
  * `differential_equations_gw.dll`
  * `dynamic_link.dll`
  * `dynamic_link_gw.dll`
  * `eispack_f.dll`
  * `elementary_functions.dll`
  * `elementary_functions_f.dll`
  * `elementary_functions_gw.dll`
  * `external_objects.dll`
  * `fftw.dll`
  * `fileio.dll`
  * `fileio_gw.dll`
  * `functions_manager.dll`
  * `GetWindowsVersion.dll`
  * `graphics.dll`
  * `hdf5.dll`
  * `hdf5_hl.dll`
  * `history_browser.dll`
  * `history_manager_gw.dll`
  * `integer_gw.dll`
  * `io.dll`
  * `io_gw.dll`
  * `libifcoremd.dll`
  * `libintl.dll`
  * `libiomp5md.dll`
  * `libjvm.dll`
  * `libmmd.dll`
  * `lapack.dll`
  * `libcurl.dll`
  * `libxml2.dll`
  * `linear_algebra.dll`
  * `linear_algebra_f.dll`
  * `linear_algebra_gw.dll`
  * `linpack_f.dll`
  * `localization_gw.dll`
  * `output_stream.dll`
  * `output_stream_f.dll`
  * `output_stream_gw.dll`
  * `pcre.dll`
  * `polynomials.dll`
  * `polynomials_f.dll`
  * `preferences.dll`
  * `tclsci.dll`
  * `threads.dll`
  * `time.dll`
  * `time_gw.dll`
  * `tk85.dll`
  * `windows_tools.dll`
  * `xml.dll`
  * `zlib1.dll`
  * `scicommons.dll`
  * `scicompletion.dll`
  * `sciconsole.dll`
  * `scigraphic_objects.dll`
  * `scigui.dll`
  * `scihistory_manager.dll`
  * `scilab_windows.dll`
  * `scilocalization.dll`
  * `scirenderer.dll`
  * `sciui_data.dll`
  * `slatec_f.dll`
  * `slicot_f.dll`
  * `sparse.dll`
  * `sparse_f.dll`
  * `sparse_gw.dll`
  * `string.dll`
  * `string_gw.dll`
  * `szip.dll`
  * `tcl85.dll`
  
* GDAL (MIT, https://gdal.org/download.html, https://anaconda.org/conda-forge/gdal) 
  via OSGEO4W (https://download.osgeo.org/osgeo4w/v2/) and GISInternals (http://download.gisinternals.com/release.php)
  * `expat.dll`
  * `freexl.dll`
  * `gdal300.dll`
  * `geos.dll`
  * `geos_c.dll`
  * `iconv.dll`
  * `iconv-2.dll`
  * `libcrypto-1_1-x64.dll`
  * `liblzma.dll`
  * `libmysql.dll`
  * `libpng16.dll`
  * `libpq.dll`
  * `libssl-1_1-x64.dll`
  * `lwgeom.dll`
  * `netcdf.dll`
  * `ogdi.dll`
  * `openjp2.dll`
  * `proj_6_3.dll`
  * `spatialite.dll`
  * `sqlite3.dll`
  * `xerces-c_3_2.dll`
  * `zstd.dll`

 
* Python (Python license, https://www.python.org/) 
  * `python36.dll`
  * `python37.dll`
  * `python38.dll`
  * `python39.dll`

### License text for scilab:

```
Scilab is a free software, and starting with version 6.0.0 beta 1, is 
released under the terms of the GNU General Public License (GPL) v2.0,
which you can find below.

Prior to this version, Scilab was licensed under the terms of the CeCILL v2.1,
and continues to be available under such terms, which you also can find below. 
Pursuant to article 5.3.4 of the CeCILL v2.1, Scilab's entire code is now 
distributed under the terms of the GNU GPL 2.0.

If during the software installation steps, you opt to select the Intel MKL 
library and/or the FFTW3 library provided in the Intel MKL library, the 
installation and use of the Intel MKL library is subject to the Intel EULA [1]
to which you then agree to be bound.The installation of Scilab also installs the
Java SE Runtime Environment (JRE) which is bound to the license agreement [2].
By installing Scilab, you agree to be bound by this Java SE license agreement 
for the installation and use of the JRE on your computer.

[1]: https://software.intel.com/en-us/articles/intel-software-development-products-license-agreement
[2]: http://www.oracle.com/technetwork/java/javase/terms/license/index.html

Some files in Scilab are licensed under a BSD License, which you can find
in the COPYING-BSD file.

Scilab is also using some codes with other licenses. Please see 
SCI/modules/*/license.txt for details.


====


                    GNU GENERAL PUBLIC LICENSE
                       Version 2, June 1991

 Copyright (C) 1989, 1991 Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.

                            Preamble

  The licenses for most software are designed to take away your
freedom to share and change it.  By contrast, the GNU General Public
License is intended to guarantee your freedom to share and change free
software--to make sure the software is free for all its users.  This
General Public License applies to most of the Free Software
Foundation's software and to any other program whose authors commit to
using it.  (Some other Free Software Foundation software is covered by
the GNU Lesser General Public License instead.)  You can apply it to
your programs, too.

  When we speak of free software, we are referring to freedom, not
price.  Our General Public Licenses are designed to make sure that you
have the freedom to distribute copies of free software (and charge for
this service if you wish), that you receive source code or can get it
if you want it, that you can change the software or use pieces of it
in new free programs; and that you know you can do these things.

  To protect your rights, we need to make restrictions that forbid
anyone to deny you these rights or to ask you to surrender the rights.
These restrictions translate to certain responsibilities for you if you
distribute copies of the software, or if you modify it.

  For example, if you distribute copies of such a program, whether
gratis or for a fee, you must give the recipients all the rights that
you have.  You must make sure that they, too, receive or can get the
source code.  And you must show them these terms so they know their
rights.

  We protect your rights with two steps: (1) copyright the software, and
(2) offer you this license which gives you legal permission to copy,
distribute and/or modify the software.

  Also, for each author's protection and ours, we want to make certain
that everyone understands that there is no warranty for this free
software.  If the software is modified by someone else and passed on, we
want its recipients to know that what they have is not the original, so
that any problems introduced by others will not reflect on the original
authors' reputations.

  Finally, any free program is threatened constantly by software
patents.  We wish to avoid the danger that redistributors of a free
program will individually obtain patent licenses, in effect making the
program proprietary.  To prevent this, we have made it clear that any
patent must be licensed for everyone's free use or not licensed at all.

  The precise terms and conditions for copying, distribution and
modification follow.

                    GNU GENERAL PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

  0. This License applies to any program or other work which contains
a notice placed by the copyright holder saying it may be distributed
under the terms of this General Public License.  The "Program", below,
refers to any such program or work, and a "work based on the Program"
means either the Program or any derivative work under copyright law:
that is to say, a work containing the Program or a portion of it,
either verbatim or with modifications and/or translated into another
language.  (Hereinafter, translation is included without limitation in
the term "modification".)  Each licensee is addressed as "you".

Activities other than copying, distribution and modification are not
covered by this License; they are outside its scope.  The act of
running the Program is not restricted, and the output from the Program
is covered only if its contents constitute a work based on the
Program (independent of having been made by running the Program).
Whether that is true depends on what the Program does.

  1. You may copy and distribute verbatim copies of the Program's
source code as you receive it, in any medium, provided that you
conspicuously and appropriately publish on each copy an appropriate
copyright notice and disclaimer of warranty; keep intact all the
notices that refer to this License and to the absence of any warranty;
and give any other recipients of the Program a copy of this License
along with the Program.

You may charge a fee for the physical act of transferring a copy, and
you may at your option offer warranty protection in exchange for a fee.

  2. You may modify your copy or copies of the Program or any portion
of it, thus forming a work based on the Program, and copy and
distribute such modifications or work under the terms of Section 1
above, provided that you also meet all of these conditions:

    a) You must cause the modified files to carry prominent notices
    stating that you changed the files and the date of any change.

    b) You must cause any work that you distribute or publish, that in
    whole or in part contains or is derived from the Program or any
    part thereof, to be licensed as a whole at no charge to all third
    parties under the terms of this License.

    c) If the modified program normally reads commands interactively
    when run, you must cause it, when started running for such
    interactive use in the most ordinary way, to print or display an
    announcement including an appropriate copyright notice and a
    notice that there is no warranty (or else, saying that you provide
    a warranty) and that users may redistribute the program under
    these conditions, and telling the user how to view a copy of this
    License.  (Exception: if the Program itself is interactive but
    does not normally print such an announcement, your work based on
    the Program is not required to print an announcement.)

These requirements apply to the modified work as a whole.  If
identifiable sections of that work are not derived from the Program,
and can be reasonably considered independent and separate works in
themselves, then this License, and its terms, do not apply to those
sections when you distribute them as separate works.  But when you
distribute the same sections as part of a whole which is a work based
on the Program, the distribution of the whole must be on the terms of
this License, whose permissions for other licensees extend to the
entire whole, and thus to each and every part regardless of who wrote it.

Thus, it is not the intent of this section to claim rights or contest
your rights to work written entirely by you; rather, the intent is to
exercise the right to control the distribution of derivative or
collective works based on the Program.

In addition, mere aggregation of another work not based on the Program
with the Program (or with a work based on the Program) on a volume of
a storage or distribution medium does not bring the other work under
the scope of this License.

  3. You may copy and distribute the Program (or a work based on it,
under Section 2) in object code or executable form under the terms of
Sections 1 and 2 above provided that you also do one of the following:

    a) Accompany it with the complete corresponding machine-readable
    source code, which must be distributed under the terms of Sections
    1 and 2 above on a medium customarily used for software interchange; or,

    b) Accompany it with a written offer, valid for at least three
    years, to give any third party, for a charge no more than your
    cost of physically performing source distribution, a complete
    machine-readable copy of the corresponding source code, to be
    distributed under the terms of Sections 1 and 2 above on a medium
    customarily used for software interchange; or,

    c) Accompany it with the information you received as to the offer
    to distribute corresponding source code.  (This alternative is
    allowed only for noncommercial distribution and only if you
    received the program in object code or executable form with such
    an offer, in accord with Subsection b above.)

The source code for a work means the preferred form of the work for
making modifications to it.  For an executable work, complete source
code means all the source code for all modules it contains, plus any
associated interface definition files, plus the scripts used to
control compilation and installation of the executable.  However, as a
special exception, the source code distributed need not include
anything that is normally distributed (in either source or binary
form) with the major components (compiler, kernel, and so on) of the
operating system on which the executable runs, unless that component
itself accompanies the executable.

If distribution of executable or object code is made by offering
access to copy from a designated place, then offering equivalent
access to copy the source code from the same place counts as
distribution of the source code, even though third parties are not
compelled to copy the source along with the object code.

  4. You may not copy, modify, sublicense, or distribute the Program
except as expressly provided under this License.  Any attempt
otherwise to copy, modify, sublicense or distribute the Program is
void, and will automatically terminate your rights under this License.
However, parties who have received copies, or rights, from you under
this License will not have their licenses terminated so long as such
parties remain in full compliance.

  5. You are not required to accept this License, since you have not
signed it.  However, nothing else grants you permission to modify or
distribute the Program or its derivative works.  These actions are
prohibited by law if you do not accept this License.  Therefore, by
modifying or distributing the Program (or any work based on the
Program), you indicate your acceptance of this License to do so, and
all its terms and conditions for copying, distributing or modifying
the Program or works based on it.

  6. Each time you redistribute the Program (or any work based on the
Program), the recipient automatically receives a license from the
original licensor to copy, distribute or modify the Program subject to
these terms and conditions.  You may not impose any further
restrictions on the recipients' exercise of the rights granted herein.
You are not responsible for enforcing compliance by third parties to
this License.

  7. If, as a consequence of a court judgment or allegation of patent
infringement or for any other reason (not limited to patent issues),
conditions are imposed on you (whether by court order, agreement or
otherwise) that contradict the conditions of this License, they do not
excuse you from the conditions of this License.  If you cannot
distribute so as to satisfy simultaneously your obligations under this
License and any other pertinent obligations, then as a consequence you
may not distribute the Program at all.  For example, if a patent
license would not permit royalty-free redistribution of the Program by
all those who receive copies directly or indirectly through you, then
the only way you could satisfy both it and this License would be to
refrain entirely from distribution of the Program.

If any portion of this section is held invalid or unenforceable under
any particular circumstance, the balance of the section is intended to
apply and the section as a whole is intended to apply in other
circumstances.

It is not the purpose of this section to induce you to infringe any
patents or other property right claims or to contest validity of any
such claims; this section has the sole purpose of protecting the
integrity of the free software distribution system, which is
implemented by public license practices.  Many people have made
generous contributions to the wide range of software distributed
through that system in reliance on consistent application of that
system; it is up to the author/donor to decide if he or she is willing
to distribute software through any other system and a licensee cannot
impose that choice.

This section is intended to make thoroughly clear what is believed to
be a consequence of the rest of this License.

  8. If the distribution and/or use of the Program is restricted in
certain countries either by patents or by copyrighted interfaces, the
original copyright holder who places the Program under this License
may add an explicit geographical distribution limitation excluding
those countries, so that distribution is permitted only in or among
countries not thus excluded.  In such case, this License incorporates
the limitation as if written in the body of this License.

  9. The Free Software Foundation may publish revised and/or new versions
of the General Public License from time to time.  Such new versions will
be similar in spirit to the present version, but may differ in detail to
address new problems or concerns.

Each version is given a distinguishing version number.  If the Program
specifies a version number of this License which applies to it and "any
later version", you have the option of following the terms and conditions
either of that version or of any later version published by the Free
Software Foundation.  If the Program does not specify a version number of
this License, you may choose any version ever published by the Free Software
Foundation.

  10. If you wish to incorporate parts of the Program into other free
programs whose distribution conditions are different, write to the author
to ask for permission.  For software which is copyrighted by the Free
Software Foundation, write to the Free Software Foundation; we sometimes
make exceptions for this.  Our decision will be guided by the two goals
of preserving the free status of all derivatives of our free software and
of promoting the sharing and reuse of software generally.

                            NO WARRANTY

  11. BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY
FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW.  EXCEPT WHEN
OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES
PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED
OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE RISK AS
TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.  SHOULD THE
PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
REPAIR OR CORRECTION.

  12. IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING
WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR
REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES,
INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED
TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY
YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER
PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.

                     END OF TERMS AND CONDITIONS

            How to Apply These Terms to Your New Programs

  If you develop a new program, and you want it to be of the greatest
possible use to the public, the best way to achieve this is to make it
free software which everyone can redistribute and change under these terms.

  To do so, attach the following notices to the program.  It is safest
to attach them to the start of each source file to most effectively
convey the exclusion of warranty; and each file should have at least
the "copyright" line and a pointer to where the full notice is found.

    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

Also add information on how to contact you by electronic and paper mail.

If the program is interactive, make it output a short notice like this
when it starts in an interactive mode:

    Gnomovision version 69, Copyright (C) year name of author
    Gnomovision comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.

The hypothetical commands `show w' and `show c' should show the appropriate
parts of the General Public License.  Of course, the commands you use may
be called something other than `show w' and `show c'; they could even be
mouse-clicks or menu items--whatever suits your program.

You should also get your employer (if you work as a programmer) or your
school, if any, to sign a "copyright disclaimer" for the program, if
necessary.  Here is a sample; alter the names:

  Yoyodyne, Inc., hereby disclaims all copyright interest in the program
  `Gnomovision' (which makes passes at compilers) written by James Hacker.

  <signature of Ty Coon>, 1 April 1989
  Ty Coon, President of Vice

This General Public License does not permit incorporating your program into
proprietary programs.  If your program is a subroutine library, you may
consider it more useful to permit linking proprietary applications with the
library.  If this is what you want to do, use the GNU Lesser General
Public License instead of this License.



====


  CeCILL FREE SOFTWARE LICENSE AGREEMENT

Version 2.1 dated 2013-06-21


    Notice

This Agreement is a Free Software license agreement that is the result
of discussions between its authors in order to ensure compliance with
the two main principles guiding its drafting:

  * firstly, compliance with the principles governing the distribution
    of Free Software: access to source code, broad rights granted to users,
  * secondly, the election of a governing law, French law, with which it
    is conformant, both as regards the law of torts and intellectual
    property law, and the protection that it offers to both authors and
    holders of the economic rights over software.

The authors of the CeCILL (for Ce[a] C[nrs] I[nria] L[ogiciel] L[ibre]) 
license are: 

Commissariat a l'energie atomique et aux energies alternatives - CEA, a
public scientific, technical and industrial research establishment,
having its principal place of business at 25 rue Leblanc, immeuble Le
Ponant D, 75015 Paris, France.

Centre National de la Recherche Scientifique - CNRS, a public scientific
and technological establishment, having its principal place of business
at 3 rue Michel-Ange, 75794 Paris cedex 16, France.

Institut National de Recherche en Informatique et en Automatique -
Inria, a public scientific and technological establishment, having its
principal place of business at Domaine de Voluceau, Rocquencourt, BP
105, 78153 Le Chesnay cedex, France.


    Preamble

The purpose of this Free Software license agreement is to grant users
the right to modify and redistribute the software governed by this
license within the framework of an open source distribution model.

The exercising of this right is conditional upon certain obligations for
users so as to preserve this status for all subsequent redistributions.

In consideration of access to the source code and the rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty and the software's author, the holder of the
economic rights, and the successive licensors only have limited liability.

In this respect, the risks associated with loading, using, modifying
and/or developing or reproducing the software by the user are brought to
the user's attention, given its Free Software status, which may make it
complicated to use, with the result that its use is reserved for
developers and experienced professionals having in-depth computer
knowledge. Users are therefore encouraged to load and test the
suitability of the software as regards their requirements in conditions
enabling the security of their systems and/or data to be ensured and,
more generally, to use and operate it in the same conditions of
security. This Agreement may be freely reproduced and published,
provided it is not altered, and that no provisions are either added or
removed herefrom.

This Agreement may apply to any or all software for which the holder of
the economic rights decides to submit the use thereof to its provisions.

Frequently asked questions can be found on the official website of the
CeCILL licenses family (http://www.cecill.info/index.en.html) for any 
necessary clarification.


    Article 1 - DEFINITIONS

For the purpose of this Agreement, when the following expressions
commence with a capital letter, they shall have the following meaning:

Agreement: means this license agreement, and its possible subsequent
versions and annexes.

Software: means the software in its Object Code and/or Source Code form
and, where applicable, its documentation, "as is" when the Licensee
accepts the Agreement.

Initial Software: means the Software in its Source Code and possibly its
Object Code form and, where applicable, its documentation, "as is" when
it is first distributed under the terms and conditions of the Agreement.

Modified Software: means the Software modified by at least one
Contribution.

Source Code: means all the Software's instructions and program lines to
which access is required so as to modify the Software.

Object Code: means the binary files originating from the compilation of
the Source Code.

Holder: means the holder(s) of the economic rights over the Initial
Software.

Licensee: means the Software user(s) having accepted the Agreement.

Contributor: means a Licensee having made at least one Contribution.

Licensor: means the Holder, or any other individual or legal entity, who
distributes the Software under the Agreement.

Contribution: means any or all modifications, corrections, translations,
adaptations and/or new functions integrated into the Software by any or
all Contributors, as well as any or all Internal Modules.

Module: means a set of sources files including their documentation that
enables supplementary functions or services in addition to those offered
by the Software.

External Module: means any or all Modules, not derived from the
Software, so that this Module and the Software run in separate address
spaces, with one calling the other when they are run.

Internal Module: means any or all Module, connected to the Software so
that they both execute in the same address space.

GNU GPL: means the GNU General Public License version 2 or any
subsequent version, as published by the Free Software Foundation Inc.

GNU Affero GPL: means the GNU Affero General Public License version 3 or
any subsequent version, as published by the Free Software Foundation Inc.

EUPL: means the European Union Public License version 1.1 or any
subsequent version, as published by the European Commission.

Parties: mean both the Licensee and the Licensor.

These expressions may be used both in singular and plural form.


    Article 2 - PURPOSE

The purpose of the Agreement is the grant by the Licensor to the
Licensee of a non-exclusive, transferable and worldwide license for the
Software as set forth in Article 5 <#scope> hereinafter for the whole
term of the protection granted by the rights over said Software.


    Article 3 - ACCEPTANCE

3.1 The Licensee shall be deemed as having accepted the terms and
conditions of this Agreement upon the occurrence of the first of the
following events:

  * (i) loading the Software by any or all means, notably, by
    downloading from a remote server, or by loading from a physical medium;
  * (ii) the first time the Licensee exercises any of the rights granted
    hereunder.

3.2 One copy of the Agreement, containing a notice relating to the
characteristics of the Software, to the limited warranty, and to the
fact that its use is restricted to experienced users has been provided
to the Licensee prior to its acceptance as set forth in Article 3.1
<#accepting> hereinabove, and the Licensee hereby acknowledges that it
has read and understood it.


    Article 4 - EFFECTIVE DATE AND TERM


      4.1 EFFECTIVE DATE

The Agreement shall become effective on the date when it is accepted by
the Licensee as set forth in Article 3.1 <#accepting>.


      4.2 TERM

The Agreement shall remain in force for the entire legal term of
protection of the economic rights over the Software.


    Article 5 - SCOPE OF RIGHTS GRANTED

The Licensor hereby grants to the Licensee, who accepts, the following
rights over the Software for any or all use, and for the term of the
Agreement, on the basis of the terms and conditions set forth hereinafter.

Besides, if the Licensor owns or comes to own one or more patents
protecting all or part of the functions of the Software or of its
components, the Licensor undertakes not to enforce the rights granted by
these patents against successive Licensees using, exploiting or
modifying the Software. If these patents are transferred, the Licensor
undertakes to have the transferees subscribe to the obligations set
forth in this paragraph.


      5.1 RIGHT OF USE

The Licensee is authorized to use the Software, without any limitation
as to its fields of application, with it being hereinafter specified
that this comprises:

 1. permanent or temporary reproduction of all or part of the Software
    by any or all means and in any or all form.

 2. loading, displaying, running, or storing the Software on any or all
    medium.

 3. entitlement to observe, study or test its operation so as to
    determine the ideas and principles behind any or all constituent
    elements of said Software. This shall apply when the Licensee
    carries out any or all loading, displaying, running, transmission or
    storage operation as regards the Software, that it is entitled to
    carry out hereunder.


      5.2 ENTITLEMENT TO MAKE CONTRIBUTIONS

The right to make Contributions includes the right to translate, adapt,
arrange, or make any or all modifications to the Software, and the right
to reproduce the resulting software.

The Licensee is authorized to make any or all Contributions to the
Software provided that it includes an explicit notice that it is the
author of said Contribution and indicates the date of the creation thereof.


      5.3 RIGHT OF DISTRIBUTION

In particular, the right of distribution includes the right to publish,
transmit and communicate the Software to the general public on any or
all medium, and by any or all means, and the right to market, either in
consideration of a fee, or free of charge, one or more copies of the
Software by any means.

The Licensee is further authorized to distribute copies of the modified
or unmodified Software to third parties according to the terms and
conditions set forth hereinafter.


        5.3.1 DISTRIBUTION OF SOFTWARE WITHOUT MODIFICATION

The Licensee is authorized to distribute true copies of the Software in
Source Code or Object Code form, provided that said distribution
complies with all the provisions of the Agreement and is accompanied by:

 1. a copy of the Agreement,

 2. a notice relating to the limitation of both the Licensor's warranty
    and liability as set forth in Articles 8 and 9,

and that, in the event that only the Object Code of the Software is
redistributed, the Licensee allows effective access to the full Source
Code of the Software for a period of at least three years from the
distribution of the Software, it being understood that the additional
acquisition cost of the Source Code shall not exceed the cost of the
data transfer.


        5.3.2 DISTRIBUTION OF MODIFIED SOFTWARE

When the Licensee makes a Contribution to the Software, the terms and
conditions for the distribution of the resulting Modified Software
become subject to all the provisions of this Agreement.

The Licensee is authorized to distribute the Modified Software, in
source code or object code form, provided that said distribution
complies with all the provisions of the Agreement and is accompanied by:

 1. a copy of the Agreement,

 2. a notice relating to the limitation of both the Licensor's warranty
    and liability as set forth in Articles 8 and 9,

and, in the event that only the object code of the Modified Software is
redistributed,

 3. a note stating the conditions of effective access to the full source
    code of the Modified Software for a period of at least three years
    from the distribution of the Modified Software, it being understood
    that the additional acquisition cost of the source code shall not
    exceed the cost of the data transfer.


        5.3.3 DISTRIBUTION OF EXTERNAL MODULES

When the Licensee has developed an External Module, the terms and
conditions of this Agreement do not apply to said External Module, that
may be distributed under a separate license agreement.


        5.3.4 COMPATIBILITY WITH OTHER LICENSES

The Licensee can include a code that is subject to the provisions of one
of the versions of the GNU GPL, GNU Affero GPL and/or EUPL in the
Modified or unmodified Software, and distribute that entire code under
the terms of the same version of the GNU GPL, GNU Affero GPL and/or EUPL.

The Licensee can include the Modified or unmodified Software in a code
that is subject to the provisions of one of the versions of the GNU GPL,
GNU Affero GPL and/or EUPL and distribute that entire code under the
terms of the same version of the GNU GPL, GNU Affero GPL and/or EUPL.


    Article 6 - INTELLECTUAL PROPERTY


      6.1 OVER THE INITIAL SOFTWARE

The Holder owns the economic rights over the Initial Software. Any or
all use of the Initial Software is subject to compliance with the terms
and conditions under which the Holder has elected to distribute its work
and no one shall be entitled to modify the terms and conditions for the
distribution of said Initial Software.

The Holder undertakes that the Initial Software will remain ruled at
least by this Agreement, for the duration set forth in Article 4.2 <#term>.


      6.2 OVER THE CONTRIBUTIONS

The Licensee who develops a Contribution is the owner of the
intellectual property rights over this Contribution as defined by
applicable law.


      6.3 OVER THE EXTERNAL MODULES

The Licensee who develops an External Module is the owner of the
intellectual property rights over this External Module as defined by
applicable law and is free to choose the type of agreement that shall
govern its distribution.


      6.4 JOINT PROVISIONS

The Licensee expressly undertakes:

 1. not to remove, or modify, in any manner, the intellectual property
    notices attached to the Software;

 2. to reproduce said notices, in an identical manner, in the copies of
    the Software modified or not.

The Licensee undertakes not to directly or indirectly infringe the
intellectual property rights on the Software of the Holder and/or
Contributors, and to take, where applicable, vis-a-vis its staff, any
and all measures required to ensure respect of said intellectual
property rights of the Holder and/or Contributors.


    Article 7 - RELATED SERVICES

7.1 Under no circumstances shall the Agreement oblige the Licensor to
provide technical assistance or maintenance services for the Software.

However, the Licensor is entitled to offer this type of services. The
terms and conditions of such technical assistance, and/or such
maintenance, shall be set forth in a separate instrument. Only the
Licensor offering said maintenance and/or technical assistance services
shall incur liability therefore.

7.2 Similarly, any Licensor is entitled to offer to its licensees, under
its sole responsibility, a warranty, that shall only be binding upon
itself, for the redistribution of the Software and/or the Modified
Software, under terms and conditions that it is free to decide. Said
warranty, and the financial terms and conditions of its application,
shall be subject of a separate instrument executed between the Licensor
and the Licensee.


    Article 8 - LIABILITY

8.1 Subject to the provisions of Article 8.2, the Licensee shall be
entitled to claim compensation for any direct loss it may have suffered
from the Software as a result of a fault on the part of the relevant
Licensor, subject to providing evidence thereof.

8.2 The Licensor's liability is limited to the commitments made under
this Agreement and shall not be incurred as a result of in particular:
(i) loss due the Licensee's total or partial failure to fulfill its
obligations, (ii) direct or consequential loss that is suffered by the
Licensee due to the use or performance of the Software, and (iii) more
generally, any consequential loss. In particular the Parties expressly
agree that any or all pecuniary or business loss (i.e. loss of data,
loss of profits, operating loss, loss of customers or orders,
opportunity cost, any disturbance to business activities) or any or all
legal proceedings instituted against the Licensee by a third party,
shall constitute consequential loss and shall not provide entitlement to
any or all compensation from the Licensor.


    Article 9 - WARRANTY

9.1 The Licensee acknowledges that the scientific and technical
state-of-the-art when the Software was distributed did not enable all
possible uses to be tested and verified, nor for the presence of
possible defects to be detected. In this respect, the Licensee's
attention has been drawn to the risks associated with loading, using,
modifying and/or developing and reproducing the Software which are
reserved for experienced users.

The Licensee shall be responsible for verifying, by any or all means,
the suitability of the product for its requirements, its good working
order, and for ensuring that it shall not cause damage to either persons
or properties.

9.2 The Licensor hereby represents, in good faith, that it is entitled
to grant all the rights over the Software (including in particular the
rights set forth in Article 5 <#scope>).

9.3 The Licensee acknowledges that the Software is supplied "as is" by
the Licensor without any other express or tacit warranty, other than
that provided for in Article 9.2 <#good-faith> and, in particular,
without any warranty as to its commercial value, its secured, safe,
innovative or relevant nature.

Specifically, the Licensor does not warrant that the Software is free
from any error, that it will operate without interruption, that it will
be compatible with the Licensee's own equipment and software
configuration, nor that it will meet the Licensee's requirements.

9.4 The Licensor does not either expressly or tacitly warrant that the
Software does not infringe any third party intellectual property right
relating to a patent, software or any other property right. Therefore,
the Licensor disclaims any and all liability towards the Licensee
arising out of any or all proceedings for infringement that may be
instituted in respect of the use, modification and redistribution of the
Software. Nevertheless, should such proceedings be instituted against
the Licensee, the Licensor shall provide it with technical and legal
expertise for its defense. Such technical and legal expertise shall be
decided on a case-by-case basis between the relevant Licensor and the
Licensee pursuant to a memorandum of understanding. The Licensor
disclaims any and all liability as regards the Licensee's use of the
name of the Software. No warranty is given as regards the existence of
prior rights over the name of the Software or as regards the existence
of a trademark.


    Article 10 - TERMINATION

10.1 In the event of a breach by the Licensee of its obligations
hereunder, the Licensor may automatically terminate this Agreement
thirty (30) days after notice has been sent to the Licensee and has
remained ineffective.

10.2 A Licensee whose Agreement is terminated shall no longer be
authorized to use, modify or distribute the Software. However, any
licenses that it may have granted prior to termination of the Agreement
shall remain valid subject to their having been granted in compliance
with the terms and conditions hereof.


    Article 11 - MISCELLANEOUS


      11.1 EXCUSABLE EVENTS

Neither Party shall be liable for any or all delay, or failure to
perform the Agreement, that may be attributable to an event of force
majeure, an act of God or an outside cause, such as defective
functioning or interruptions of the electricity or telecommunications
networks, network paralysis following a virus attack, intervention by
government authorities, natural disasters, water damage, earthquakes,
fire, explosions, strikes and labor unrest, war, etc.

11.2 Any failure by either Party, on one or more occasions, to invoke
one or more of the provisions hereof, shall under no circumstances be
interpreted as being a waiver by the interested Party of its right to
invoke said provision(s) subsequently.

11.3 The Agreement cancels and replaces any or all previous agreements,
whether written or oral, between the Parties and having the same
purpose, and constitutes the entirety of the agreement between said
Parties concerning said purpose. No supplement or modification to the
terms and conditions hereof shall be effective as between the Parties
unless it is made in writing and signed by their duly authorized
representatives.

11.4 In the event that one or more of the provisions hereof were to
conflict with a current or future applicable act or legislative text,
said act or legislative text shall prevail, and the Parties shall make
the necessary amendments so as to comply with said act or legislative
text. All other provisions shall remain effective. Similarly, invalidity
of a provision of the Agreement, for any reason whatsoever, shall not
cause the Agreement as a whole to be invalid.


      11.5 LANGUAGE

The Agreement is drafted in both French and English and both versions
are deemed authentic.


    Article 12 - NEW VERSIONS OF THE AGREEMENT

12.1 Any person is authorized to duplicate and distribute copies of this
Agreement.

12.2 So as to ensure coherence, the wording of this Agreement is
protected and may only be modified by the authors of the License, who
reserve the right to periodically publish updates or new versions of the
Agreement, each with a separate number. These subsequent versions may
address new issues encountered by Free Software.

12.3 Any Software distributed under a given version of the Agreement may
only be subsequently distributed under the same version of the Agreement
or a subsequent version, subject to the provisions of Article 5.3.4
<#compatibility>.


    Article 13 - GOVERNING LAW AND JURISDICTION

13.1 The Agreement is governed by French law. The Parties agree to
endeavor to seek an amicable solution to any disagreements or disputes
that may arise during the performance of the Agreement.

13.2 Failing an amicable solution within two (2) months as from their
occurrence, and unless emergency proceedings are necessary, the
disagreements or disputes shall be referred to the Paris Courts having
jurisdiction, by the more diligent Party.
```


### License text for GDAL/OSGeo4W:
```
Copyright (c) [multiple years] [multiple copyright holders]

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
```
### License text for GISInternals Suite: 
```
Copyright (c) 2020, Tamas Szekeres

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
```
### License text for Python: 
```
PSF LICENSE AGREEMENT FOR PYTHON 3.9.6

1. This LICENSE AGREEMENT is between the Python Software Foundation ("PSF"), and
   the Individual or Organization ("Licensee") accessing and otherwise using Python
   3.9.6 software in source or binary form and its associated documentation.

2. Subject to the terms and conditions of this License Agreement, PSF hereby
   grants Licensee a nonexclusive, royalty-free, world-wide license to reproduce,
   analyze, test, perform and/or display publicly, prepare derivative works,
   distribute, and otherwise use Python 3.9.6 alone or in any derivative
   version, provided, however, that PSF's License Agreement and PSF's notice of
   copyright, i.e., "Copyright © 2001-2021 Python Software Foundation; All Rights
   Reserved" are retained in Python 3.9.6 alone or in any derivative version
   prepared by Licensee.

3. In the event Licensee prepares a derivative work that is based on or
   incorporates Python 3.9.6 or any part thereof, and wants to make the
   derivative work available to others as provided herein, then Licensee hereby
   agrees to include in any such work a brief summary of the changes made to Python
   3.9.6.

4. PSF is making Python 3.9.6 available to Licensee on an "AS IS" basis.
   PSF MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS OR IMPLIED.  BY WAY OF
   EXAMPLE, BUT NOT LIMITATION, PSF MAKES NO AND DISCLAIMS ANY REPRESENTATION OR
   WARRANTY OF MERCHANTABILITY OR FITNESS FOR ANY PARTICULAR PURPOSE OR THAT THE
   USE OF PYTHON 3.9.6 WILL NOT INFRINGE ANY THIRD PARTY RIGHTS.

5. PSF SHALL NOT BE LIABLE TO LICENSEE OR ANY OTHER USERS OF PYTHON 3.9.6
   FOR ANY INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES OR LOSS AS A RESULT OF
   MODIFYING, DISTRIBUTING, OR OTHERWISE USING PYTHON 3.9.6, OR ANY DERIVATIVE
   THEREOF, EVEN IF ADVISED OF THE POSSIBILITY THEREOF.

6. This License Agreement will automatically terminate upon a material breach of
   its terms and conditions.

7. Nothing in this License Agreement shall be deemed to create any relationship
   of agency, partnership, or joint venture between PSF and Licensee.  This License
   Agreement does not grant permission to use PSF trademarks or trade name in a
   trademark sense to endorse or promote products or services of Licensee, or any
   third party.

8. By copying, installing or otherwise using Python 3.9.6, Licensee agrees
   to be bound by the terms and conditions of this License Agreement.
```

### Other external libraries

Some external libraries (e.g. used by GDAL and scilabs) are under radically different licenses, you MUST obtain valid licenses for each of these dependent libraries.
See the license information of the respective libraries included in their downloads for further information.