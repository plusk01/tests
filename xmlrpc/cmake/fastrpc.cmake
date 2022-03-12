# version 8.1.10 - 5 March 2022
set(FASTRPC_MAJOR 8)
set(FASTRPC_MINOR 1)
set(FASTRPC_REVISION 10)
set(FASTRPC_VERSION ${FASTRPC_MAJOR}.${FASTRPC_MINOR}.${FASTRPC_REVISION})

find_package(LibXml2 REQUIRED)

include(FetchContent)
FetchContent_Declare(fastrpc
    GIT_REPOSITORY https://github.com/seznam/fastrpc
    GIT_TAG "v${FASTRPC_VERSION}")
# FetchContent_MakeAvailable(fastrpc)

FetchContent_GetProperties(fastrpc)
if(NOT fastrpc_POPULATED)
    FetchContent_Populate(fastrpc)

    configure_file(${fastrpc_SOURCE_DIR}/src/frpcversion.h.in ${fastrpc_SOURCE_DIR}/src/frpcversion.h @ONLY)

    add_library(fastrpc SHARED)
    target_link_libraries(fastrpc LibXml2::LibXml2)
    target_include_directories(fastrpc PUBLIC ${fastrpc_SOURCE_DIR}/src)
    target_sources(fastrpc PRIVATE
        ${fastrpc_SOURCE_DIR}/src/frpcarray.cc
        ${fastrpc_SOURCE_DIR}/src/frpcb64unmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcb64writer.cc
        ${fastrpc_SOURCE_DIR}/src/frpcbase64.cc
        ${fastrpc_SOURCE_DIR}/src/frpcbinary.cc
        ${fastrpc_SOURCE_DIR}/src/frpcbinmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcbinunmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcbool.cc
        ${fastrpc_SOURCE_DIR}/src/frpc.cc
        ${fastrpc_SOURCE_DIR}/src/frpccompare.cc
        ${fastrpc_SOURCE_DIR}/src/frpcconfig.cc
        ${fastrpc_SOURCE_DIR}/src/frpcconnector.cc
        ${fastrpc_SOURCE_DIR}/src/frpcdatabuilder.cc
        ${fastrpc_SOURCE_DIR}/src/frpcdatetime.cc
        ${fastrpc_SOURCE_DIR}/src/frpcdouble.cc
        ${fastrpc_SOURCE_DIR}/src/frpcencodingerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcfault.cc
        ${fastrpc_SOURCE_DIR}/src/frpchttp.cc
        ${fastrpc_SOURCE_DIR}/src/frpchttpclient.cc
        ${fastrpc_SOURCE_DIR}/src/frpchttperror.cc
        ${fastrpc_SOURCE_DIR}/src/frpchttpio.cc
        ${fastrpc_SOURCE_DIR}/src/frpcindexerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcint.cc
        ${fastrpc_SOURCE_DIR}/src/frpcjsonmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpckeyerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpclenerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcmethodregistry.cc
        ${fastrpc_SOURCE_DIR}/src/frpcnull.cc
        ${fastrpc_SOURCE_DIR}/src/frpcpool.cc
        ${fastrpc_SOURCE_DIR}/src/frpcprotocolerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcresponseerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcserver.cc
        ${fastrpc_SOURCE_DIR}/src/frpcserverproxy.cc
        ${fastrpc_SOURCE_DIR}/src/frpcstreamerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcstring.cc
        ${fastrpc_SOURCE_DIR}/src/frpcstring_view.cc
        ${fastrpc_SOURCE_DIR}/src/frpcstruct.cc
        ${fastrpc_SOURCE_DIR}/src/frpctreebuilder.cc
        ${fastrpc_SOURCE_DIR}/src/frpctreefeeder.cc
        ${fastrpc_SOURCE_DIR}/src/frpctypeerror.cc
        ${fastrpc_SOURCE_DIR}/src/frpcunmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcurlunmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcvalue.cc
        ${fastrpc_SOURCE_DIR}/src/frpcwriter.cc
        ${fastrpc_SOURCE_DIR}/src/frpcxmlmarshaller.cc
        ${fastrpc_SOURCE_DIR}/src/frpcxmlunmarshaller.cc)
endif()