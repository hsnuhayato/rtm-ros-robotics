#include <stdio.h>
#include <xmlrpc-c/client.h>

int main(int const argc, const char ** const argv) {
    xmlrpc_env env;
    xmlrpc_value * resultP;
    int sum;
    char * const url = "http://xmlrpc-c.sourceforge.net/api/sample.php";
    char * const methodName = "sample.add";

    /* Initialize our error-handling environment. */
    xmlrpc_env_init(&env);
    /* Start up our XML-RPC client library. */
    xmlrpc_client_init2(&env, XMLRPC_CLIENT_NO_FLAGS, "Client", "1.0", NULL, 0);
    /* Make the remote procedure call */
    resultP = xmlrpc_client_call(&env, url, methodName,  "(ii)", (xmlrpc_int32) 1, (xmlrpc_int32) 2);
    /* Get our state name and print it out. */
    xmlrpc_parse_value(&env, resultP, "i", &sum);
    printf("The sum  is %d\n", sum);
    /* Dispose of our result value. */
    xmlrpc_DECREF(resultP);
    /* Clean up our error-handling environment. */
    xmlrpc_env_clean(&env);
    /* Shutdown our XML-RPC client library. */
    xmlrpc_client_cleanup();
    return 0;
}

