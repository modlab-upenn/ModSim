import React from 'react';
import {Redirect} from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Index(): React.JSX.Element {
  return <Redirect to={useBaseUrl('/docs/')} />;
}
